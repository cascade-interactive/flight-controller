/*
 * main_bridge.cpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#include "main_bridge.h"
#include "CoreMath.hpp"
#include "cmsis_os.h"
#include "imu_icm20602.hpp"

extern SPI_HandleTypeDef hspi1;

void flight_controller_run(void) {

    // HW Init
    ICM20602_IMU imu(&hspi1, SPI1_IMU_CS_GPIO_Port, SPI1_IMU_CS_Pin);
    imu.begin();

    while (1) {

        osDelay(1);
    }
}