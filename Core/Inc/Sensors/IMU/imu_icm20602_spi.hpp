/*
 * imu_icm20602_spi.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */
#pragma once

#include "CoreMath.hpp"
#include "imu_icm20602.hpp"

ICM20602_IMU::ICM20602_IMU(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) : _hspi(hspi), _cs_port(cs_port), _cs_pin(cs_pin) {

    // Clear on startup
    gyro = CoreMath::Vector3::Zero();
    accel = CoreMath::Vector3::Zero();
}

void ICM20602_IMU::select_cs() {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
}

void ICM20602_IMU::deselect_cs() {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
}
