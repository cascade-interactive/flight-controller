/*
 * imu_icm20602.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */
#pragma once

#include "CoreMath.hpp"
#include "cmsis_os.h"
#include "imu_icm20602_registers.hpp"
#include "main.h"

#ifndef INC_SENSORS_IMU_IMU_ICM20602_HPP_
#define INC_SENSORS_IMU_IMU_ICM20602_HPP_

class ICM20602_IMU {
  private:
    SPI_HandleTypeDef *_hspi;
    GPIO_TypeDef *_cs_port;
    uint16_t _cs_pin;

    uint8_t _raw_buffer[6];

    void select_cs();
    void deselect_cs();

    HAL_StatusTypeDef write_register(uint8_t reg_addr, uint8_t data);
    HAL_StatusTypeDef read_registers(uint8_t reg_addr, uint8_t *p_buffer, uint16_t size);

    // DMA
    HAL_StatusTypeDef read_registers_dma(uint8_t reg_addr, uint8_t *p_buffer, uint16_t size);

    uint8_t _dma_tx_buffer[16];
    uint8_t _dma_rx_buffer[16];
    volatile bool _dma_done;
    volatile bool _dma_busy;
    uint8_t *_dma_user_buffer;
    uint16_t _dma_user_size;

  public:
    CoreMath::Vector3 gyro;
    CoreMath::Vector3 accel;

    // Constructor
    ICM20602_IMU(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

    bool begin();
    void update_gyro();
    void update_accel();

    void on_spi_dma_complete(SPI_HandleTypeDef *hspi);
    bool dma_done() const;

    HAL_StatusTypeDef start_accel_dma();
    HAL_StatusTypeDef start_gyro_dma();
    void parse_accel();
    void parse_gyro();
};

#endif /* INC_SENSORS_IMU_IMU_ICM20602_HPP_ */
