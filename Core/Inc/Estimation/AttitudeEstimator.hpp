/*
 * imu_icm20602_registers.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#pragma once
#include "CoreMath.hpp"
#define CM CoreMath

class AttitudeEstimator {
  public:
    void Reset();
    void Update(const CM::Vector3 &gyro_rad_s, const CM::Vector3 &accel_g, float dt);
};