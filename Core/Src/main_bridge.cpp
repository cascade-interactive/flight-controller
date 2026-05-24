/*
 * main_bridge.cpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#include "main_bridge.h"
#include "CoreMath.hpp"
#include "Time.hpp"
#include "cmsis_os.h"
#include "imu_icm20602.hpp"
#include "usbd_cdc_if.h"
#include <cstdio>
#include <cstring>

extern SPI_HandleTypeDef hspi1;

void flight_controller_run(void) {
    osDelay(5);

    // Initialize the IMU
    ICM20602_IMU imu(&hspi1, SPI1_IMU_CS_GPIO_Port, SPI1_IMU_CS_Pin);
    bool imu_ok = imu.begin();

    uint32_t previous_timestamp_us = 0U;
    bool imu_dma_active = false;

    // DMA STEP 1
    if (imu_ok) {
        imu_dma_active = (imu.start_sample_dma() == HAL_OK);
    }

    while (1) {
        // Check if the DMA hardware finishes grabbing a new sample
        if (imu_ok && imu_dma_active && imu.sample_ready()) {

            // Consume the data and reset the DMA flag
            ICM20602Sample sample = imu.consume_sample();

            // Calculate float dt for estimator integration
            float dt_s = Time::get_dt_seconds(previous_timestamp_us, sample.timestamp_us);
            previous_timestamp_us = sample.timestamp_us;

            // estimator.update(sample.gyro_rad_s, sample.accel_g, dt_s);
            // CoreMath::Vector3 current_attitude = estimator.get_euler_attitude();

            // CoreMath::Vector3 desired_attitude = rc_receiver.get_setpoints();
            // float throttle = rc_receiver.get_throttle();

            // CoreMath::Vector3 pid_output = pid_controller.compute(desired_attitude, current_attitude, sample.gyro_rad_s, dt_s);

            // motor_mixer.mix_and_output(throttle, pid_output);

            // Tell DMA to start fetching the next sample behind the scenes
            if (imu.start_sample_dma() != HAL_OK) {
                imu_dma_active = false; // Flag it if it fails
            }
        }

        // If DMA stopped or failed to start, try to start it again
        if (imu_ok && !imu_dma_active) {
            imu_dma_active = (imu.start_sample_dma() == HAL_OK);
        }

        osDelay(1);
    }
}
