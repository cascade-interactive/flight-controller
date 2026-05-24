# STM32 Flight Controller

Custom flight-controller firmware for an STM32G474CEU6-based quadcopter, with the near-term goal of stable hover and the longer-term goal of becoming a small autonomy platform.

This repo is currently in hardware bring-up and sensor/estimator development.

## Current Status

- STM32G474CEU6 project generated with CubeMX
- FreeRTOS enabled for task scheduling
- USB CDC serial debug output working
- C/C++ bridge in place for flight-controller logic
- ICM-20602 SPI communication working
- ICM-20602 `WHO_AM_I` check working
- Accelerometer and gyroscope burst reads implemented
- Experimental non-blocking SPI DMA read path in progress
- Basic vector and quaternion math utilities started
- Attitude estimator started, not yet implemented

## Hardware Target

| Component | Role | Interface | Status |
| --- | --- | --- | --- |
| STM32G474CEU6 | Main MCU, 170 MHz Cortex-M4F with FPU | - | Project target |
| ICM-20602 | 6-axis IMU, accel + gyro | SPI | Bring-up in progress |
| VL53L1X | Time-of-flight range sensor for altitude hold | I2C | Planned |
| PMW3901 | Optical-flow sensor for horizontal hold | SPI | Planned |
| Raspberry Pi 5 | Higher-level autonomy companion computer | UART | Planned |

The STM32G474 was chosen for its FPU and for the CORDIC/FMAC accelerators, which should be useful later for estimator and control-loop math.

## Firmware Structure

The generated CubeMX files remain in place for MCU startup, HAL setup, USB, and FreeRTOS integration. The flight-controller logic is kept separate from that generated code where possible.

## ICM-20602

The IMU driver currently handles:

- Manual chip-select control
- Register writes over SPI
- Single and burst register reads over SPI
- `WHO_AM_I` validation
- I2C interface disable
- Gyro full-scale setup at +/-250 dps
- Accelerometer full-scale setup at +/-2g
- Raw accel/gyro conversion into physical units
- Experimental SPI transmit/receive DMA reads

The DMA path is still being worked through. The current loop alternates between accel and gyro DMA reads, parses the completed transfer, and prints the values over USB CDC for debug.

## Goals

### Current

- Finish robust SPI and I2C sensor drivers
- Complete ICM-20602 bring-up and calibration
- Add VL53L1X ranging
- Add PMW3901 optical-flow reads
- Implement attitude estimation
- Reach stable hover

### Planned

- Position hold using optical flow and ToF
- UART link to Raspberry Pi 5 for higher-level autonomy
- Waypoint navigation
