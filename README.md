# STM32 Bare-Metal CRSF Receiver → RGB LEDs

Register-level C firmware (no HAL, no Arduino core) for an **STM32F446RE Nucleo-64** that reads stick positions from a **RadioMaster ExpressLRS receiver** over **CRSF** and maps them to RGB LED brightness.

▶️ **Demo video:** TODO

## What it does

| Stick | Output |
|---|---|
| Roll | External RGB LED: Red |
| Pitch | External RGB LED: Blue |
| Throttle | External RGB LED: Green |
| Yaw | On-board Nucleo LED (LD2) |

## How it works

1. **Clock and pin setup.** Enables the GPIOA and USART1 peripheral clocks through RCC. Configures PA0/PA1/PA4/PA5 as push-pull outputs and routes PA9/PA10 to USART1 (alternate function 7).
2. **UART at 420 kbaud.** CRSF runs at a non-standard 420,000 baud. With the 16 MHz internal clock and 8x oversampling, BRR = `0x0046`, which gives about 421 kbaud (0.25% error).
3. **Interrupt-driven receive.** The USART1 RX interrupt (IRQ 37) pushes each byte into a 256-byte ring buffer. The ISR only writes `rx_head`, and the main loop only writes `rx_tail`, so the two sides never contend for the same index.
4. **Frame parsing.** The main loop scans for the CRSF sync byte (`0xC8`), checks the length (24) and frame type (`0x16`, RC channels), and validates the CRC8 (DVB-S2, polynomial `0xD5`) before trusting the data.
5. **Channel unpacking.** It unpacks the 11-bit channel values from the packed payload with bit shifts and masks.
6. **Software PWM.** It scales each channel (172–1811) to a 0–100 duty cycle, then drives the pins with atomic writes to the GPIO BSRR register.

## Wiring

| Nucleo header | MCU pin | Function |
|---|---|---|
| D2 (CN9) | PA10 | USART1 RX ← receiver TX |
| D8 (CN5) | PA9 | USART1 TX → receiver RX |
| A0 (CN8) | PA0 | Red LED |
| A1 (CN8) | PA1 | Green LED |
| A2 (CN8) | PA4 | Blue LED |
| on-board | PA5 | LD2 (yaw) |

Receiver powered from the Nucleo's 5V and GND. UART lines cross over: receiver TX goes to MCU RX.

## Hardware

- STM32 Nucleo-64 (NUCLEO-F446RE)
- RadioMaster ExpressLRS receiver bound to a RadioMaster Pocket transmitter
- Common-cathode RGB LED with current-limiting resistors

## Development notes

Built with LLM-assisted development: I gave the model the STM32F446 datasheet and the RM0390 reference manual and iterated on the firmware with it. The firmware worked end to end on first integration.

## Possible improvements

- Replace software PWM with hardware timer PWM, so brightness doesn't depend on loop timing.
- Add a failsafe that dims or blinks the LEDs if no valid frame arrives within ~100 ms, instead of holding the last values.
- Use DMA with idle-line detection instead of a per-byte interrupt.
- Decode all 16 channels and use the aux switches for mode changes.
