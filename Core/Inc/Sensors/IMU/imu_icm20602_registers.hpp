/*
 * imu_icm20602_registers.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */
#pragma once

#ifndef INC_SENSORS_IMU_IMU_ICM20602_REGISTERS_HPP_
#define INC_SENSORS_IMU_IMU_ICM20602_REGISTERS_HPP_

#include <stdint.h>

// Register definitions
namespace ICM20602 {
	namespace Reg {
		inline constexpr uint8_t WHO_AM_I		= 0x75;
		inline constexpr uint8_t CONFIG			= 0x1A;	// Page 36

		inline constexpr uint8_t GYRO_CONFIG	= 0x1B;

		inline constexpr uint8_t ACCEL_CONFIG	= 0x1C;
		inline constexpr uint8_t ACCEL_CONFIG_2	= 0x1D;

		inline constexpr uint8_t PWR_MGMT_1		= 0x6B; // Page 46
		inline constexpr uint8_t PWR_MGMT_2		= 0x6C;

		inline constexpr uint8_t I2C_IF			= 0x70;	// Bit 4 disables I2C


		// Readings
		inline constexpr uint8_t GYRO_XOUT_H	= 0x43; // 6 bytes, spanning 0x43 to 0x48
		inline constexpr uint8_t ACCEL_XOUT_H	= 0x3B; // 6 bytes, spanning 0x3B to 0x40
	}
}

#endif /* INC_SENSORS_IMU_IMU_ICM20602_REGISTERS_HPP_ */
