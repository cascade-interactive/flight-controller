# STM32 Flight Controller
A flight controller built around the STM32G474CEU6 with the goal of achieving stable hover. The SPI and I2C drivers are written from scratch using the datasheets and register maps directly. Clean code is a priority.
This is also the base for a broader autonomy platform I'm working toward.

## Hardware
ComponentRoleInterfaceSTM32G474CEU6Main MCU (170 MHz, Cortex-M4F + FPU)ICM-206026-axis IMU (accel + gyro)SPIVL53L1XTime-of-flight ranging for altitude holdI2CPMW3901Optical flow for horizontal position holdSPI
The G474 was chosen for its FPU and the CORDIC and FMAC hardware accelerators, which will be useful for filter math down the line.

## Firmware
The SPI and I2C drivers go straight to registers from the datasheets rather than using library drivers. This keeps the behavior of those peripherals predictable and means I actually know what's happening at every step.
ICM-20602 -- Register-level init: DLPF, sample rate divider, gyro/accel full-scale range, burst sensor data reads.
VL53L1X -- Full initialization via ST's register map without their driver blob. The VL53L1X has a non-trivial init sequence and a 16-bit register address space, both handled explicitly.
PMW3901 -- Register init and delta-X/delta-Y burst reads for optical flow.
For attitude estimation, Madgwick or Mahony filter is planned. No autotune, at least not yet.

## Goals
### Current

- SPI and I2C drivers
- ICM-20602 bring-up and calibration
 - VL53L1X ranging
 - PMW3901 optical flow reads
 - Attitude estimation and stable hover

### Planned

- Position hold using optical flow + ToF
 -UART link to Raspberry Pi 5 for higher-level autonomy
- Waypoint navigation


## Code Style

C17, compiled with -Wall -Wextra -Werror
Register addresses and bit masks in dedicated headers, named after datasheet notation
Functions do one thing
