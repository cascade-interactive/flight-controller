# Flight Controller

A quadcopter flight controller I'm building from scratch, firmware and hardware both. The whole point was to actually teach myself how real-time embedded systems like drones work, so I stayed away from using LLMs even for the trivial stuff. That forced me to sit down and understand the things I would've otherwise skipped: how an EKF actually works, how a cascaded PID loop comes together, how to get a sensor talking cleanly over SPI without hoping for the best.

![STM32 flight controller](documentation/fc_showcase.png)

![Platform](https://img.shields.io/badge/MCU-STM32G474RET6-03234B?logo=stmicroelectronics&logoColor=white)
![Language](https://img.shields.io/badge/Firmware-C%20%2F%20C%2B%2B-00599C?logo=cplusplus&logoColor=white)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-1BA027)
![Toolchain](https://img.shields.io/badge/Toolchain-STM32CubeIDE-03234B)
![Status](https://img.shields.io/badge/status-active-brightgreen)
![License](https://img.shields.io/badge/license-CC%20BY--NC%204.0-lightgrey)

> To see the full build log go to my site: [alexanderbugar.com](https://alexanderbugar.com)

## Where it's at

Right now it's in the bring-up and estimator phase, running on a **WeAct STM32G474** breadboard with an **ICM-20602**, since that's what I had on hand to prototype with. The custom PCB it's actually meant to run on is a separate project, and once that board arrives the firmware ports over to it.

The thing I was happiest about recently was getting a deterministic **1 ms IMU loop** running clean. It's fully DMA driven, so the CPU never sits there blocking on the SPI bus. I fire off a read, poll a sample-ready flag, consume the sample when the hardware's done, and immediately re-arm the next one so it's already on its way before I'm finished with the current sample.

The other one was finally having automated tests instead of flashing firmware and squinting at a serial terminal. I wired in Google Test and wrote a few checks for the estimator: that it starts at the identity quaternion, stays normalized, and doesn't spit out NaNs under simple conditions. Nothing fancy, but it beats testing everything on hardware.

**Working**

- STM32 project generated with CubeMX, FreeRTOS task scheduling
- USB CDC serial debug streaming
- C/C++ bridge so the flight-controller code sits apart from the generated files
- ICM-20602 SPI driver: `WHO_AM_I` validation, accel/gyro burst reads, DMA reads
- Deterministic 1 ms DMA-driven IMU loop
- Vector and quaternion math utilities
- EKF first revision + passing unit tests (Google Test)

**In progress**

- Sensor calibration
- Attitude estimation wired into the loop
- Cascaded PID control

## Hardware

| Component      | Role                        | Interface | Status                 |
| -------------- | --------------------------- | --------- | ---------------------- |
| STM32G474RET6  | Main MCU                    | -         | Current PCB target     |
| STM32G474CEU6  | Previous 48-pin MCU package | -         | Legacy configuration   |
| ICM-20602      | 6-axis IMU                  | SPI       | Working (prototype)    |
| ICM-42605-P    | 6-axis IMU                  | SPI       | Planned (custom board) |
| Barometer      | Altitude, external module   | I2C       | Planned                |
| VL53L1X        | Time-of-flight range sensor | I2C       | Planned                |
| PMW3901        | Optical-flow sensor         | SPI       | Planned                |
| Raspberry Pi 5 | Higher-level autonomy       | UART      | Planned                |

I picked the STM32G4 for the FPU plus the CORDIC and FMAC accelerators, which are genuinely useful for the estimator and control-loop math. The IMU stays deliberately simple for now. While I'm debugging SPI, calibration, and estimation, fewer things that can go wrong is a good thing.

## The board

The firmware is meant to run on a custom flight controller PCB I'm designing in Altium instead of a generic board I'd have to bend the code around. It's an all-in-one design that regulates power from the battery on-board and drives the motors with an FOC ESC. I'm putting some of the schematic PDFs in `hardware/` as I go, and the routing once it's done.

### Moving to the 64-pin package

I ended up switching the custom board from the 48-pin **STM32G474CEU6** to the 64-pin LQFP **STM32G474RET6**. It is the same STM32G4 family, but the larger package gives the design more exposed pins and more room for peripherals. Since I don't want to spend the time and money on multiple iterations of essentially the same board, it made more sense to add that headroom now instead of designing myself into a corner.

The new package gives me:

- one additional UART, allowing up to five serial devices simultaneously
- two additional PWM outputs for servos
- three more unrestricted GPIOs

The CubeMX configuration, generated firmware, linker scripts, startup code, and pinout documentation now all target the STM32G474RE in LQFP64. Once the pinout is confirmed, it is back to routing and finalizing the design.

Part of me still feels like I'm finding reasons not to order the board, but at some point I have to commit instead of continuing to add things and getting stuck in analysis paralysis.

## Repository layout

| Folder           | What's in it                                                        |
| ---------------- | ------------------------------------------------------------------- |
| `firmware/`      | The STM32CubeIDE project: CubeMX config, drivers, estimator, USB    |
| `hardware/`      | Schematics, and PCB files as the board work lands here              |
| `documentation/` | The pinout diagram and other docs                                   |
| `tools/`         | Small helper scripts for the repo                                   |

The old STM32G4 firmware from before the restructure is preserved on the [`legacy/lynn-fc`](../../tree/legacy/lynn-fc) branch.
The previous STM32G474CEU6 board configuration is preserved on the [`legacy/g474ceu`](../../tree/legacy/g474ceu) branch.

## Pinout

The STM32G474RE LQFP64 diagram and table below are generated automatically from the CubeMX `.ioc`, so they can't drift out of sync with the actual configuration.

<!-- PINOUT:BEGIN (auto-generated by tools/pinout/generate_pinout.py, do not edit by hand) -->

![Pinout](documentation/pinout.svg)

| Pin | Signal | Function | Bus |
|-----|--------|----------|-----|
| PA0 | `ADC1_IN1` | V_SENSE | ANALOG |
| PA1 | `S_TIM2_CH2` | PWM_1 | PWM · TIM2 |
| PA2 | `USART2_TX` | USART2_TX | USART2 |
| PA3 | `USART2_RX` | USART2_RX | USART2 |
| PA4 | `ADC2_IN17` | CUR_SENSE | ANALOG |
| PA5 | `S_TIM2_CH1` | PWM_2 | PWM · TIM2 |
| PA6 | `S_TIM3_CH1` | MOTOR_CH1 | MOTORS · TIM3 |
| PA7 | `S_TIM3_CH2` | MOTOR_CH2 | MOTORS · TIM3 |
| PA8 | `GPIO_Output` | LED_ERROR | LEDS |
| PA9 | `GPIO_Output` | FSYNC | SPI2 · IMU |
| PA11 | `USB_DM` | USB_DM | USB |
| PA12 | `USB_DP` | USB_DP | USB |
| PA13 | `SYS_JTMS-SWDIO` | SYS_SWDIO | SWD |
| PA14 | `SYS_JTCK-SWCLK` | SYS_SWCLK | SWD |
| PA15 | `I2C1_SCL` | I2C1_SCL | I2C1 · TOF |
| PB0 | `S_TIM3_CH3` | MOTOR_CH3 | MOTORS · TIM3 |
| PB1 | `S_TIM3_CH4` | MOTOR_CH4 | MOTORS · TIM3 |
| PB2 | `GPXTI2` | SPI1_INT | SPI1 · EXT |
| PB3 | `SPI1_SCK` | SPI1_SCK | SPI1 · EXT |
| PB4 | `SPI1_MISO` | SPI1_MISO | SPI1 · EXT |
| PB5 | `SPI1_MOSI` | SPI1_MOSI | SPI1 · EXT |
| PB6 | `GPIO_Output` | SPI1_CS | SPI1 · EXT |
| PB7 | `I2C1_SDA` | I2C1_SDA | I2C1 · TOF |
| PB8 | `-` | BOOT0 | System |
| PB9 | `FDCAN1_TX` | FDCAN1_TX | Other |
| PB10 | `USART3_TX` | USART3_TX | USART3 |
| PB11 | `USART3_RX` | USART3_RX | USART3 |
| PB12 | `GPIO_Output` | SPI2_CS | SPI2 · IMU |
| PB13 | `SPI2_SCK` | SPI2_SCK | SPI2 · IMU |
| PB14 | `SPI2_MISO` | SPI2_MISO | SPI2 · IMU |
| PB15 | `SPI2_MOSI` | SPI2_MOSI | SPI2 · IMU |
| PC0 | `LPUART1_RX` | ESC_TELEM | LPUART1 |
| PC1 | `LPUART1_TX` | LPUART1_TX | LPUART1 |
| PC2 | `S_TIM1_CH3` | SERV_1 | SERVOS · TIM1 |
| PC3 | `S_TIM1_CH4` | SERV_2 | SERVOS · TIM1 |
| PC4 | `USART1_TX` | USART1_TX | USART1 |
| PC5 | `USART1_RX` | USART1_RX | USART1 |
| PC7 | `GPIO_Output` | LED_STATUS | LEDS |
| PC8 | `GPXTI8` | SPI2_INT | SPI2 · IMU |
| PC9 | `GPIO_Output` | LED_ACTIVITY | LEDS |
| PC10 | `GPIO_Output` | XSHUT | I2C1 · TOF |
| PC12 | `UART5_TX` | UART5_TX | UART5 |
| PD2 | `UART5_RX` | UART5_RX | UART5 |
| PF0 | `RCC_OSC_IN` | RCC_OSC_IN | System |
| PF1 | `RCC_OSC_OUT` | RCC_OSC_OUT | System |
| PG10 | `-` | NRST | System |

<!-- PINOUT:END -->

## References

The resources that helped me the most, in case they're useful to anyone else going down this path:

- Joan Solà, *Quaternion kinematics for the error-state Kalman filter*
- Tellex, Brown, Lupashin, *Estimation for Quadrotors*
- James Han, [*The Extended Kalman Filter (EKF): Why Taylor Expansions are Awesome*](https://www.youtube.com/watch?v=9X3jGGnbcvU) (YouTube)

## Links

- **Site / build log:** [alexanderbugar.com](https://alexanderbugar.com)
- **GitHub:** [cascade-interactive](https://github.com/cascade-interactive)
- **LinkedIn:** [Alexander Bugar](https://www.linkedin.com/in/alexander-bugar-0569a0226/)

## License

This is my portfolio work. Feel free to read through it, learn from it, and use it in your own non-commercial projects, just credit me. Please don't sell it or use it in anything commercial. Formally that's [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/), see [LICENSE](LICENSE).
