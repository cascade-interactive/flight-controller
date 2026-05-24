/*
 * imu_icm20602_spi.cpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#include "CoreMath.hpp"
#include "Time.hpp"
#include "imu_icm20602.hpp"
#include "imu_icm20602_registers.hpp"

using namespace CoreMath;

static ICM20602_IMU *active_imu = nullptr;

ICM20602_IMU::ICM20602_IMU(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) : _hspi(hspi), _cs_port(cs_port), _cs_pin(cs_pin) {

    active_imu = this;

    // DMA
    _dma_done = false;
    _dma_busy = false;
    _dma_user_buffer = nullptr;
    _dma_user_size = 0;

    // Clear on startup
    _last_sample_timestamp_us = 0;
    gyro = Vector3::Zero();
    accel = Vector3::Zero();

    for (uint8_t &byte : _raw_buffer) {
        byte = 0;
    }

    for (uint8_t &byte : _raw_sample_buffer) {
        byte = 0;
    }
}

void ICM20602_IMU::select_cs() {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
}

void ICM20602_IMU::deselect_cs() {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef ICM20602_IMU::write_register(uint8_t reg_addr, uint8_t data) {
    HAL_StatusTypeDef status;

    uint8_t transmit_data[2] = {reg_addr, data};

    select_cs();
    status = HAL_SPI_Transmit(_hspi, transmit_data, 2, HAL_MAX_DELAY);
    deselect_cs();

    return status;
}

HAL_StatusTypeDef ICM20602_IMU::read_registers(uint8_t reg_addr, uint8_t *p_buffer, uint16_t size) {
    HAL_StatusTypeDef status;

    uint8_t transmit_addr = reg_addr | 0x80; // MSB high

    select_cs();

    status = HAL_SPI_Transmit(_hspi, &transmit_addr, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(_hspi, p_buffer, size, HAL_MAX_DELAY);
    }
    deselect_cs();

    return status;
}

HAL_StatusTypeDef ICM20602_IMU::read_registers_dma(uint8_t reg_addr, uint8_t *p_buffer, uint16_t size) {
    if (_dma_busy) {
        return HAL_BUSY;
    }

    if (size + 1 > sizeof(_dma_tx_buffer)) {
        return HAL_ERROR;
    }

    _dma_done = false;
    _dma_busy = true;
    _dma_user_buffer = p_buffer;
    _dma_user_size = size;

    _dma_tx_buffer[0] = reg_addr | 0x80;

    for (uint16_t i = 1; i < size + 1; i++) {
        _dma_tx_buffer[i] = 0x00;
    }

    select_cs();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(_hspi, _dma_tx_buffer, _dma_rx_buffer, size + 1);

    if (status != HAL_OK) {
        deselect_cs();
        _dma_busy = false;
        return status;
    }

    return HAL_OK;
}

void ICM20602_IMU::on_spi_dma_complete(SPI_HandleTypeDef *hspi) {
    if (hspi != _hspi) {
        return;
    }

    for (uint16_t i = 0; i < _dma_user_size; i++) {
        _dma_user_buffer[i] = _dma_rx_buffer[i + 1];
    }

    deselect_cs();

    _dma_done = true;
    _dma_busy = false;

    _last_sample_timestamp_us = Time::Micros();
}

bool ICM20602_IMU::dma_done() const {
    return _dma_done;
}

HAL_StatusTypeDef ICM20602_IMU::start_accel_dma() {
    return read_registers_dma(ICM20602::Reg::ACCEL_XOUT_H, _raw_buffer, 6);
}

HAL_StatusTypeDef ICM20602_IMU::start_gyro_dma() {
    return read_registers_dma(ICM20602::Reg::GYRO_XOUT_H, _raw_buffer, 6);
}

/*
Burst combined DMA IMU read starting at ACCEL_XOUT_H:

ACCEL_XOUT_H/L
ACCEL_YOUT_H/L
ACCEL_ZOUT_H/L
TEMP_OUT_H/L
GYRO_XOUT_H/L
GYRO_YOUT_H/L
GYRO_ZOUT_H/L

14 bytes total
*/
HAL_StatusTypeDef ICM20602_IMU::start_sample_dma() {
    return read_registers_dma(ICM20602::Reg::ACCEL_XOUT_H, _raw_sample_buffer, 14);
}

ICM20602Sample ICM20602_IMU::consume_sample() {
    if (!_dma_done) {
        return {};
    }

    ICM20602Sample sample = parse_sample();
    _dma_done = false;
    return sample;
}

bool ICM20602_IMU::sample_ready() const {
    return _dma_done;
}

ICM20602Sample ICM20602_IMU::parse_sample() const {
    ICM20602Sample sample{};

    int16_t raw_ax = static_cast<int16_t>((_raw_sample_buffer[0] << 8) | _raw_sample_buffer[1]);
    int16_t raw_ay = static_cast<int16_t>((_raw_sample_buffer[2] << 8) | _raw_sample_buffer[3]);
    int16_t raw_az = static_cast<int16_t>((_raw_sample_buffer[4] << 8) | _raw_sample_buffer[5]);

    int16_t raw_gx = static_cast<int16_t>((_raw_sample_buffer[8] << 8) | _raw_sample_buffer[9]);
    int16_t raw_gy = static_cast<int16_t>((_raw_sample_buffer[10] << 8) | _raw_sample_buffer[11]);
    int16_t raw_gz = static_cast<int16_t>((_raw_sample_buffer[12] << 8) | _raw_sample_buffer[13]);

    sample.accel_g =
        CoreMath::Vector3(static_cast<float>(raw_ax) / 16384.0f, static_cast<float>(raw_ay) / 16384.0f, static_cast<float>(raw_az) / 16384.0f);

    sample.gyro_rad_s = CoreMath::Vector3((static_cast<float>(raw_gx) / 131.0f) * CoreMath::DegToRad,
                                          (static_cast<float>(raw_gy) / 131.0f) * CoreMath::DegToRad,
                                          (static_cast<float>(raw_gz) / 131.0f) * CoreMath::DegToRad);

    sample.timestamp_us = _last_sample_timestamp_us;
    sample.valid = true;

    return sample;
}

void ICM20602_IMU::parse_accel() {
    int16_t raw_x = (static_cast<int16_t>(_raw_buffer[0] << 8) | _raw_buffer[1]);
    int16_t raw_y = (static_cast<int16_t>(_raw_buffer[2] << 8) | _raw_buffer[3]);
    int16_t raw_z = (static_cast<int16_t>(_raw_buffer[4] << 8) | _raw_buffer[5]);

    accel.x = static_cast<float>(raw_x) / 16384.0f;
    accel.y = static_cast<float>(raw_y) / 16384.0f;
    accel.z = static_cast<float>(raw_z) / 16384.0f;
}

void ICM20602_IMU::parse_gyro() {
    int16_t raw_x = (static_cast<int16_t>(_raw_buffer[0] << 8) | _raw_buffer[1]);
    int16_t raw_y = (static_cast<int16_t>(_raw_buffer[2] << 8) | _raw_buffer[3]);
    int16_t raw_z = (static_cast<int16_t>(_raw_buffer[4] << 8) | _raw_buffer[5]);

    gyro.x = static_cast<float>(raw_x) / 131.0f;
    gyro.y = static_cast<float>(raw_y) / 131.0f;
    gyro.z = static_cast<float>(raw_z) / 131.0f;
}

bool ICM20602_IMU::begin() {
    uint8_t who_am_i_val = 0;

    read_registers(ICM20602::Reg::WHO_AM_I, &who_am_i_val, 1);
    if (who_am_i_val != ICM20602::Reg::WHO_AM_I_VALUE) {
        return false; // Failed to communicate
    }

    write_register(ICM20602::Reg::I2C_IF, 0x40);       // Disable I2C
    write_register(ICM20602::Reg::PWR_MGMT_1, 0x01);   // Clock source
    write_register(ICM20602::Reg::GYRO_CONFIG, 0x00);  // Full-Scale Range: 250 dps   (Scale factor: 131.0 LSB/dps)
    write_register(ICM20602::Reg::ACCEL_CONFIG, 0x00); // Full-Scale Range: 2g        (Scale factor: 16384.0 LSB/g)

    return true;
}

void ICM20602_IMU::update_gyro() {
    if (read_registers_dma(ICM20602::Reg::GYRO_XOUT_H, _raw_buffer, 6) == HAL_OK) {
        int16_t raw_x = (static_cast<int16_t>(_raw_buffer[0] << 8) | _raw_buffer[1]);
        int16_t raw_y = (static_cast<int16_t>(_raw_buffer[2] << 8) | _raw_buffer[3]);
        int16_t raw_z = (static_cast<int16_t>(_raw_buffer[4] << 8) | _raw_buffer[5]);

        // Convert to dps using the scale factor
        gyro.x = static_cast<float>(raw_x) / 131.0f;
        gyro.y = static_cast<float>(raw_y) / 131.0f;
        gyro.z = static_cast<float>(raw_z) / 131.0f;
    }
}

void ICM20602_IMU::update_accel() {
    if (read_registers_dma(ICM20602::Reg::ACCEL_XOUT_H, _raw_buffer, 6) == HAL_OK) {
        int16_t raw_x = (static_cast<int16_t>(_raw_buffer[0] << 8) | _raw_buffer[1]);
        int16_t raw_y = (static_cast<int16_t>(_raw_buffer[2] << 8) | _raw_buffer[3]);
        int16_t raw_z = (static_cast<int16_t>(_raw_buffer[4] << 8) | _raw_buffer[5]);

        // Convert
        accel.x = static_cast<float>(raw_x) / 16384.0f;
        accel.y = static_cast<float>(raw_y) / 16384.0f;
        accel.z = static_cast<float>(raw_z) / 16384.0f;
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (active_imu != nullptr) {
        active_imu->on_spi_dma_complete(hspi);
    }
}
