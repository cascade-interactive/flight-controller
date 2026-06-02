/*
 * AttitudeEstimator.cpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#include "AttitudeEstimator.hpp"

AttitudeEstimator::AttitudeEstimator() {
    current_attitude_ = CoreMath::Quaternion::Identity();

    // P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_[i][j] = (i == j) ? 0.1f : 0.0f;
        }
    }

    // Q using ICM-20602 datasheet noise
    float gyro_var = 1.5e-5f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Q_[i][j] = (i == j) ? gyro_var : 0.0f;
        }
    }

    // R
    float accel_var = 8.6e-3f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_[i][j] = (i == j) ? accel_var : 0.0f;
        }
    }
};

void AttitudeEstimator::predict(const CoreMath::Vector3 &gyro_rad_s, float dt_s) {
    CoreMath::Quaternion omega = CoreMath::Quaternion::FromAngularVelocity(gyro_rad_s); /* convert gyro to pure quaternion */
    CoreMath::Quaternion q_dot = current_attitude_ * omega * 0.5f;                      /* compute how fast it is changing */
    current_attitude_ = current_attitude_ + q_dot * dt_s;                               /* integrate forward */
    normalizeAttitude();

    // Covariance
    float wx = gyro_rad_s.x;
    float wy = gyro_rad_s.y;
    float wz = gyro_rad_s.z;

    float F[3][3] = {
        {0.0f, -wz, wy},
        {wz, 0.0f, -wx},
        {-wy, wx, 0.0f},
    };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[i][j] = -F[i][j];
        }
    }

    // Propagate
    float FP[3][3] = {};
    float PFt[3][3] = {};
    float Ft[3][3] = {};

    // F^T
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Ft[i][j] = F[j][i];
        }
    }

    // F*P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * P_[k][j];
            }
        }
    }

    // P*F^T
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                PFt[i][j] += P_[i][k] * Ft[k][j];
            }
        }
    }

    // Update covariance
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_[i][j] += (FP[i][j] + PFt[i][j] + Q_[i][j]) * dt_s;
        }
    }
}

void AttitudeEstimator::normalizeAttitude() {
    current_attitude_.Normalize();
}

CoreMath::Vector3 AttitudeEstimator::getEuler() const {
    return CoreMath::QuaternionToEuler(current_attitude_);
}

CoreMath::Quaternion AttitudeEstimator::getAttitude() const {
    return current_attitude_;
}

void AttitudeEstimator::updateAccel(const CoreMath::Vector3 &accel_g) {
    if (accel_g.IsNearlyZero())
        return;

    float mag = accel_g.Size();

    if (mag < 0.5f || mag > 1.5f)
        return;

    CoreMath::Vector3 accel_normalized = accel_g.Normalized();
    CoreMath::Vector3 gravity_estimate = CoreMath::RotateVector(current_attitude_, CoreMath::Vector3::Up());

    // Innovation
    CoreMath::Vector3 y = accel_normalized - gravity_estimate;

    // Innovation covariance
    float S[3][3] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P_[i][j] + R_[i][j];

    // Kalman Gain

    // Invert S (3x3)
    float detS = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                 S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

    float inv_detS = 1.0f / detS;

    float Sinv[3][3] = {};
    Sinv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * inv_detS;
    Sinv[0][1] = -(S[0][1] * S[2][2] - S[0][2] * S[2][1]) * inv_detS;
    Sinv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * inv_detS;
    Sinv[1][0] = -(S[1][0] * S[2][2] - S[1][2] * S[2][0]) * inv_detS;
    Sinv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * inv_detS;
    Sinv[1][2] = -(S[0][0] * S[1][2] - S[0][2] * S[1][0]) * inv_detS;
    Sinv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * inv_detS;
    Sinv[2][1] = -(S[0][0] * S[2][1] - S[0][1] * S[2][0]) * inv_detS;
    Sinv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * inv_detS;

    float K[3][3] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                K[i][j] += P_[i][k] * Sinv[k][j];

    // State correction converted back to quaternion form
    CoreMath::Quaternion correction(1.0f,
                                    (K[0][0] * y.x + K[0][1] * y.y + K[0][2] * y.z) * 0.5f,
                                    (K[1][0] * y.x + K[1][1] * y.y + K[1][2] * y.z) * 0.5f,
                                    (K[2][0] * y.x + K[2][1] * y.y + K[2][2] * y.z) * 0.5f);
    current_attitude_ = current_attitude_ * correction;
    normalizeAttitude();

    // Covariance update
    float I_KH[3][3] = {};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            I_KH[i][j] = (i == j) ? 1.0f : 0.0f;
            for (int k = 0; k < 3; k++) {
                I_KH[i][j] -= K[i][k] * P_[k][j];
            }
        }
    }

    float new_P[3][3] = {};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                new_P[i][j] += I_KH[i][k] * P_[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_[i][j] = new_P[i][j];
        }
    }
}

void AttitudeEstimator::update(CoreMath::Vector3 gyro_rad_s, CoreMath::Vector3 accel_g, float dt_s) {
    predict(gyro_rad_s, dt_s);
    updateAccel(accel_g);
}
