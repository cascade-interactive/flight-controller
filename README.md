# STM32 Flight Controller

The main purpose of this project were mainly to teach myself how exactly real-time embedded systems such as drones work. That's why I stayed away from using LLMs for even trivial tasks. This forced me to understand concepts such as how an EKF works, how to implement a cascaded PID loop and much more. Some of the ressources that have helped me the most I will link here:

* Joan Solà — Quaternion kinematics for the ESKF
* Estimation for Quadrotors (Tellex et al.)

## Current Status

At the moment the project is still mainly in the bring-up and estimator development phase.

Currently working:

* STM32 project generated using CubeMX
* FreeRTOS task scheduling
* USB CDC serial debug output
* C/C++ bridge for flight-controller code
* ICM-20602 SPI communication
* ICM-20602 `WHO_AM_I` validation
* Accelerometer burst reads
* Gyroscope burst reads
* Basic vector math utilities
* Basic quaternion math utilities

Currently in progress:

* SPI DMA reads
* Sensor calibration
* Attitude estimation

One thing I was pretty happy about recently was finally getting some EKF unit tests passing. Up until now most debugging was basically flashing firmware, opening a serial terminal and hoping for the best. I integrated Google Test and wrote a few basic tests to validate estimator behavior. Currently the tests verify that the filter starts at the identity quaternion, remains normalized and doesn't produce NaNs under simple conditions. Nothing crazy yet, but it was nice to finally have some automated validation instead of relying entirely on hardware testing.

## Hardware

| Component      | Role                        | Interface | Status         |
| -------------- | --------------------------- | --------- | -------------- |
| STM32F4        | Main MCU                    | -         | Current target |
| ICM-20602      | 6-axis IMU                  | SPI       | Working        |
| VL53L1X        | Time-of-flight range sensor | I2C       | Planned        |
| PMW3901        | Optical-flow sensor         | SPI       | Planned        |
| Raspberry Pi 5 | Higher-level autonomy       | UART      | Planned        |

The project originally started on an STM32G474. I picked it because of the FPU as well as the CORDIC and FMAC accelerators which seemed pretty useful for estimator and control-loop math. Later on I decided that I wanted to design my own PCB for this project and eventually assemble it myself. At that point the G4 started becoming a little difficult to justify from a cost perspective, so I switched over to an STM32F4 instead.

## Firmware Structure

The CubeMX-generated files are mostly left alone and handle startup, peripheral initialization, USB and FreeRTOS setup.

Most flight-controller-specific code lives separately from the generated code where possible.

Current modules include:

* IMU drivers
* Math utilities
* State estimation
* Control infrastructure
* Communication interfaces

## ICM-20602

The IMU driver currently supports:

* Manual chip-select control
* SPI register writes
* Single-register reads
* Burst-register reads
* `WHO_AM_I` validation
* I2C interface disable
* Accelerometer configuration
* Gyroscope configuration
* Conversion from raw sensor values into physical units
* Experimental DMA-based reads

The current debug loop alternates between accelerometer and gyroscope DMA reads, parses the completed transfer and streams the values over USB CDC.

The ICM-20602 was mainly chosen because it is simple and relatively common. Long term I'd like to switch to an IMU that includes a magnetometer, but while debugging SPI communication, calibration and estimation code I figured having fewer things that can go wrong was probably a good thing.

## Goals

### Current

* Finish SPI and I2C sensor drivers
* Complete IMU calibration
* Add VL53L1X support
* Add PMW3901 support
* Implement attitude estimation
* Implement cascaded PID control
* Reach stable hover

### Planned

* Position hold using optical flow and ToF
* UART link to a Raspberry Pi 5
* Waypoint navigation
* Basic autonomous flight capabilities
* Custom flight-controller PCB
