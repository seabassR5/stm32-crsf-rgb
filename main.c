#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * PHYSICAL-TO-SILICON PIN TRANSLATION TABLE
 * ============================================================================
 * Arduino Header | Silicon Pin | Peripheral Route | Function
 * ----------------------------------------------------------------------------
 * CN9 Pin 3 (D2) | PA10        | USART1_RX (AF07) | CRSF telemetry input
 * CN5 Pin 1 (D8) | PA9         | USART1_TX (AF07) | CRSF telemetry return
 * CN8 Pin 1 (A0) | PA0         | General Output   | External RGB: Red
 * CN8 Pin 2 (A1) | PA1         | General Output   | External RGB: Green
 * CN8 Pin 3 (A2) | PA4         | General Output   | External RGB: Blue
 * (On-board LD2) | PA5         | General Output   | Nucleo Green LED: Yaw
 * ============================================================================
 */

/* ----------------------------------------------------------------------------
 * MEMORY-MAPPED REGISTER BASES & OFFSETS
 *
 * Every peripheral on the STM32 is mapped to a fixed 32-bit physical RAM/bus
 * address defined in the STM32F446 Reference Manual (RM0390).
 * ----------------------------------------------------------------------------
 */

// 1. Reset and Clock Control (RCC) — controls power gating to chip subsystems
#define RCC_BASE            (0x40023800UL)
// AHB1ENR (+0x30): Controls clock gating for GPIO ports (Port A through H)
#define RCC_AHB1ENR         (*(volatile uint32_t *)(RCC_BASE + 0x30))
// APB2ENR (+0x44): Controls clock gating for high-speed peripherals (USART1, TIM1, etc.)
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x44))

// 2. GPIO Port A Registers
#define GPIOA_BASE          (0x40020000UL)
// MODER (+0x00): Pin direction (2 bits/pin: 00=Input, 01=Output, 10=Alternate Func, 11=Analog)
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
// PUPDR (+0x0C): Pull-up/pull-down resistors (2 bits/pin: 00=None, 01=Pull-Up, 10=Pull-Down)
#define GPIOA_PUPDR         (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
// BSRR (+0x18): Atomic Bit Set/Reset Register (Low 16 bits = Set pin HIGH; High 16 bits = Pull pin LOW)
#define GPIOA_BSRR          (*(volatile uint32_t *)(GPIOA_BASE + 0x18))
// AFRH (+0x24): Alternate Function High Register (4 bits/pin for pins PA8 through PA15)
#define GPIOA_AFRH          (*(volatile uint32_t *)(GPIOA_BASE + 0x24))

// 3. USART1 Registers
#define USART1_BASE         (0x40011000UL)
// SR (+0x00): Status Register (holds hardware event flags like "Data Received")
#define USART1_SR           (*(volatile uint32_t *)(USART1_BASE + 0x00))
// DR (+0x04): Data Register (reads incoming byte, writes outgoing byte)
#define USART1_DR           (*(volatile uint32_t *)(USART1_BASE + 0x04))
// BRR (+0x08): Baud Rate Register (clock prescaler fraction/mantissa)
#define USART1_BRR          (*(volatile uint32_t *)(USART1_BASE + 0x08))
// CR1 (+0x0C): Control Register 1 (peripheral enable, oversampling, IRQ enable)
#define USART1_CR1          (*(volatile uint32_t *)(USART1_BASE + 0x0C))

// 4. Nested Vectored Interrupt Controller (NVIC) in ARM Cortex-M4 Core
// ISER1 handles enabling Interrupt Request (IRQ) lines 32 through 63
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104UL))

/* --- Bitfield Masks --- */
#define USART_SR_RXNE       (1U << 5)   // Read Data Register Not Empty flag
#define USART_CR1_OVER8     (1U << 15)  // 8x oversampling mode flag
#define USART_CR1_UE        (1U << 13)  // USART Enable
#define USART_CR1_RXNEIE    (1U << 5)   // RX Interrupt Enable
#define USART_CR1_TE        (1U << 3)   // Transmitter Enable
#define USART_CR1_RE        (1U << 2)   // Receiver Enable

/* --- CRSF Protocol Constants --- */
#define CRSF_SYNC_BYTE      0xC8
#define CRSF_FRAMETYPE_RC   0x16
#define RX_BUFFER_SIZE      256

static volatile uint8_t  rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t          rx_tail = 0;
static volatile uint16_t channels[16];

// DVB-S2 CRC8 Checksum algorithm used by the CRSF specification
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else            crc <<= 1;
        }
    }
    return crc;
}

// Unpacks packed 11-bit channel values from CRSF payload bytes
static void crsf_unpack(const uint8_t *p) {
    channels[0] = (uint16_t)((p[0]       | p[1] << 8)                     & 0x07FF); // Roll
    channels[1] = (uint16_t)((p[1] >> 3  | p[2] << 5)                     & 0x07FF); // Pitch
    channels[2] = (uint16_t)((p[2] >> 6  | p[3] << 2 | p[4] << 10)        & 0x07FF); // Throttle
    channels[3] = (uint16_t)((p[4] >> 1  | p[5] << 7)                     & 0x07FF); // Yaw
    channels[4] = (uint16_t)((p[5] >> 4  | p[6] << 4)                     & 0x07FF); // Aux1
}

// Hardware Interrupt Vector triggered immediately when USART1 finishes reading a byte
void USART1_IRQHandler(void) {
    if (USART1_SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)(USART1_DR & 0xFF);
        uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_buffer[rx_head] = byte;
            rx_head = next_head;
        }
    }
}

// Scans circular buffer for valid CRSF frames and unpacks RC data
static void crsf_poll(void) {
    while (((rx_head - rx_tail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE) >= 26) {
        if (rx_buffer[rx_tail] != CRSF_SYNC_BYTE) {
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
            continue;
        }
        uint16_t len_idx = (rx_tail + 1) % RX_BUFFER_SIZE;
        if (rx_buffer[len_idx] != 24) {
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
            continue;
        }
        uint8_t packet[26];
        for (uint8_t i = 0; i < 26; i++) {
            packet[i] = rx_buffer[(rx_tail + i) % RX_BUFFER_SIZE];
        }
        if (packet[2] == CRSF_FRAMETYPE_RC) {
            if (crsf_crc8(&packet[2], 23) == packet[25]) {
                crsf_unpack(&packet[3]);
                rx_tail = (rx_tail + 26) % RX_BUFFER_SIZE;
                continue;
            }
        }
        rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    }
}

/* ----------------------------------------------------------------------------
 * HARDWARE INITIALIZATION
 * Connects the internal clock gates, configures pin direction, and hooks up
 * the hardware serial engine.
 * ----------------------------------------------------------------------------
 */
static void Hardware_Init(void) {
    // ------------------------------------------------------------------------
    // Step 1: Supply Clock Power to Peripherals
    // Peripherals stay cold and powered-off by default to save energy.
    // ------------------------------------------------------------------------
    RCC_AHB1ENR |= (1U << 0); // Bit 0: Connect system clock bus to GPIO Port A
    RCC_APB2ENR |= (1U << 4); // Bit 4: Connect peripheral clock bus to USART1

    // ------------------------------------------------------------------------
    // Step 2: Configure Output Pins for the LEDs
    // In MODER, each pin has a 2-bit field:
    // 00 = Input, 01 = Output, 10 = Alternate Function, 11 = Analog
    // ------------------------------------------------------------------------

    // PA5 (On-board Green LD2): Bits 11:10
    GPIOA_MODER &= ~(3U << (5 * 2)); // Clear bits 11:10
    GPIOA_MODER |=  (1U << (5 * 2)); // Write 01 (General-purpose output)

    // PA0 (CN8 Pin A0 / Red), PA1 (CN8 Pin A1 / Green), PA4 (CN8 Pin A2 / Blue)
    // Clear the two mode bits for pin 0, pin 1, and pin 4:
    GPIOA_MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (4 * 2)));
    // Write 01 into pin 0, pin 1, and pin 4 to make them push-pull outputs:
    GPIOA_MODER |=  ((1U << (0 * 2)) | (1U << (1 * 2)) | (1U << (4 * 2)));

    // ------------------------------------------------------------------------
    // Step 3: Route Serial Pins (PA9 = TX / D8, PA10 = RX / D2)
    // ------------------------------------------------------------------------
    // Set PA9 and PA10 to mode "10" (Alternate Function mode)
    GPIOA_MODER &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
    GPIOA_MODER |=  ((2U << (9 * 2)) | (2U << (10 * 2)));

    // Internal silicon multiplexer (AFRH controls pins 8-15; 4 bits per pin):
    // PA9  is bits [7:4]
    // PA10 is bits [11:8]
    // From the STM32F446 datasheet, AF07 corresponds to USART1/2/3.
    GPIOA_AFRH &= ~((0xFU << 4) | (0xFU << 8)); // Clear existing routing
    GPIOA_AFRH |=  ((7U << 4)   | (7U << 8));   // Route AF07 into PA9 and PA10

    // Enable internal pull-up resistor on PA10 (RX / D2) so it doesn't float when unplugged
    // PUPDR: 00=None, 01=Pull-Up (bits 21:20 for pin 10)
    GPIOA_PUPDR &= ~(3U << (10 * 2));
    GPIOA_PUPDR |=  (1U << (10 * 2));

    // ------------------------------------------------------------------------
    // Step 4: Configure Hardware Baud Rate for USART1 (420,000 baud)
    // Core Clock = 16 MHz HSI. With OVER8 = 1 (8x oversampling):
    // Prescaler formula: 16,000,000 / (8 * 420,000) = 4.7619
    // Integer Mantissa = 4, Fraction = round(0.7619 * 8) = 6 -> BRR = 0x0046
    // ------------------------------------------------------------------------
    USART1_BRR = 0x0046;

    // Enable USART peripheral, transmitter, receiver, 8x oversampling, and RX interrupt
    USART1_CR1 = USART_CR1_OVER8 | USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;

    // ------------------------------------------------------------------------
    // Step 5: Unmask Hardware Interrupt Vector in ARM Cortex-M4 Core
    // USART1 is IRQ #37. ISER1 covers interrupts 32 to 63: 37 - 32 = Bit 5
    // ------------------------------------------------------------------------
    NVIC_ISER1 = (1U << (37 - 32));
}

// Math conversion: maps CRSF 11-bit range (172 to 1811) to PWM duty cycle (0 to 100)
static uint8_t channel_to_duty(uint16_t ch) {
    if (ch < 172) ch = 172;
    if (ch > 1811) ch = 1811;
    return (uint8_t)(((uint32_t)(ch - 172) * 100) / (1811 - 172));
}

int main(void) {
    Hardware_Init();

    uint8_t pwm_counter = 0;
    uint8_t duty_red    = 0;
    uint8_t duty_green  = 0;
    uint8_t duty_blue   = 0;
    uint8_t duty_yaw    = 0;

    while (1) {
        // High-speed software PWM counter cycling from 0 to 99
        pwm_counter++;
        if (pwm_counter >= 100) {
            pwm_counter = 0;

            // Check for freshly decoded radio packets once every PWM period
            crsf_poll();

            // Calculate duty cycles from current stick positions
            uint8_t raw_red = channel_to_duty(channels[0]);        // Roll stick -> Red LED
            duty_red   = (uint8_t)(((uint16_t)raw_red * 80) / 100); // Scaled to 80% max to balance brightness

            duty_blue  = channel_to_duty(channels[1]);              // Pitch stick -> Blue LED
            duty_green = channel_to_duty(channels[2]);              // Throttle stick -> Green LED
            duty_yaw   = channel_to_duty(channels[3]);              // Yaw stick -> On-board LD2
        }

        /* --------------------------------------------------------------------
         * PHYSICAL PIN SWITCHING VIA BSRR (Bit Set / Reset Register)
         *
         * Writing a 1 to bit N pulls physical pin N HIGH (3.3V).
         * Writing a 1 to bit (N + 16) pulls physical pin N LOW (0V / GND).
         * Writing 0 does nothing (atomic operation, completely glitch-free).
         * --------------------------------------------------------------------
         */

        // CN8 Pin A0 / PA0 (Red)
        if (pwm_counter < duty_red) GPIOA_BSRR = (1U << 0);       // Bit 0: Set PA0 HIGH (3.3V)
        else                        GPIOA_BSRR = (1U << (0 + 16));// Bit 16: Reset PA0 to 0V

        // CN8 Pin A1 / PA1 (Green)
        if (pwm_counter < duty_green) GPIOA_BSRR = (1U << 1);     // Bit 1: Set PA1 HIGH (3.3V)
        else                          GPIOA_BSRR = (1U << (1 + 16));// Bit 17: Reset PA1 to 0V

        // CN8 Pin A2 / PA4 (Blue)
        if (pwm_counter < duty_blue) GPIOA_BSRR = (1U << 4);      // Bit 4: Set PA4 HIGH (3.3V)
        else                         GPIOA_BSRR = (1U << (4 + 16));// Bit 20: Reset PA4 to 0V

        // On-Board Green LED / PA5 (Yaw)
        if (pwm_counter < duty_yaw) GPIOA_BSRR = (1U << 5);       // Bit 5: Set PA5 HIGH (3.3V)
        else                        GPIOA_BSRR = (1U << (5 + 16));// Bit 21: Reset PA5 to 0V
    }
}
