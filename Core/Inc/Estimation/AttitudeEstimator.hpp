/*
 * AttitudeEstimator.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#pragma once
#include "CoreMath.hpp"

class AttitudeEstimator {
  public:
    AttitudeEstimator();

    // Input
    void update(CoreMath::Vector3 gyro_rad_s, CoreMath::Vector3 accel_g, float dt_s);

    // Output
    CoreMath::Quaternion getAttitude() const;
    CoreMath::Vector3 getEuler() const;

  private:
    // State
    CoreMath::Quaternion current_attitude_{CoreMath::Quaternion::Identity()};

    // Covariance
    float P_[3][3];

    // Process Noise
    float Q_[3][3];

    // Measurement Noise
    float R_[3][3];

    // Steps
    void predict(const CoreMath::Vector3 &gyro_rad_s, float dt_s);
    void updateAccel(const CoreMath::Vector3 &accel_g);
    void normalizeAttitude();
};