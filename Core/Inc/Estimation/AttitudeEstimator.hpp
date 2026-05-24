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
    AttitudeEstimator() = default;

    // Input
    void update(CoreMath::Vector3 gyro_rad_s, CoreMath::Vector3 accel_g, float dt_s);

    // Output
    CoreMath::Vector3 get_euler_attitude() const;

  private:
    // State
    CoreMath::Quaternion current_attitude{CoreMath::Quaternion::Identity()};

    // TODO: Covariance matrices for EKF
};