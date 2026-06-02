#include "AttitudeEstimator.hpp"
#include <gtest/gtest.h>

TEST(AttitudeEstimator, StartsAtIdentity) {
    AttitudeEstimator estimator;

    CoreMath::Quaternion q = estimator.getAttitude();

    EXPECT_NEAR(q.w, 1.0f, 1e-5f);
    EXPECT_NEAR(q.x, 0.0f, 1e-5f);
    EXPECT_NEAR(q.y, 0.0f, 1e-5f);
    EXPECT_NEAR(q.z, 0.0f, 1e-5f);
}

TEST(AttitudeEstimator, NoMotionStaysLevel) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.0f, 0.0f, 0.0f};
    CoreMath::Vector3 accel{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 1000; i++) {
        estimator.update(gyro, accel, 0.001f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();

    EXPECT_NEAR(euler.x, 0.0f, 1e-3f);
    EXPECT_NEAR(euler.y, 0.0f, 1e-3f);
}

TEST(AttitudeEstimator, QuaternionStaysNormalized) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.1f, -0.2f, 0.05f};
    CoreMath::Vector3 accel{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 1000; i++) {
        estimator.update(gyro, accel, 0.01f);
    }

    CoreMath::Quaternion q = estimator.getAttitude();
    float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    EXPECT_NEAR(norm, 1.0f, 1e-3f);
}

TEST(AttitudeEstimator, DoesNotProduceNaN) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.3f, 0.1f, -0.2f};
    CoreMath::Vector3 accel{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 1000; i++) {
        estimator.update(gyro, accel, 0.01f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();

    EXPECT_TRUE(std::isfinite(euler.x));
    EXPECT_TRUE(std::isfinite(euler.y));
    EXPECT_TRUE(std::isfinite(euler.z));
}

TEST(AttitudeEstimator, RollRateIntegration) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{1.0f, 0.0f, 0.0f};
    CoreMath::Vector3 accel{0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 100; i++) {
        estimator.update(gyro, accel, 0.01f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();

    EXPECT_NEAR(euler.x, 1.0f, 0.05f);
    EXPECT_NEAR(euler.y, 0.0f, 0.05f);
    EXPECT_NEAR(euler.z, 0.0f, 0.05f);
}

TEST(AttitudeEstimator, PitchRateIntegration) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.0f, 1.0f, 0.0f};
    CoreMath::Vector3 accel{0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 100; i++) {
        estimator.update(gyro, accel, 0.01f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();

    EXPECT_NEAR(euler.x, 0.0f, 0.05f);
    EXPECT_NEAR(euler.y, 1.0f, 0.05f);
    EXPECT_NEAR(euler.z, 0.0f, 0.05f);
}

TEST(AttitudeEstimator, AccelConvergesToTilt) {
    AttitudeEstimator estimator;

    float tilt = CoreMath::Pi / 4.0f;
    CoreMath::Vector3 gyro{0.0f, 0.0f, 0.0f};
    CoreMath::Vector3 accel{0.0f, std::sin(tilt), std::cos(tilt)};

    for (int i = 0; i < 5000; i++) {
        estimator.update(gyro, accel, 0.001f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();

    EXPECT_GT(euler.x, 0.3f);
    EXPECT_LT(euler.x, CoreMath::Pi / 2.0f);
    EXPECT_GT(std::abs(euler.x), std::abs(euler.y));
}

TEST(AttitudeEstimator, ZeroAccelSkipsUpdate) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.0f, 0.0f, 0.0f};
    CoreMath::Vector3 accel_valid{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 100; i++) {
        estimator.update(gyro, accel_valid, 0.01f);
    }

    CoreMath::Quaternion before = estimator.getAttitude();

    CoreMath::Vector3 accel_zero{0.0f, 0.0f, 0.0f};
    estimator.update(gyro, accel_zero, 0.01f);

    CoreMath::Quaternion after = estimator.getAttitude();

    EXPECT_NEAR(after.w, before.w, 1e-4f);
    EXPECT_NEAR(after.x, before.x, 1e-4f);
    EXPECT_NEAR(after.y, before.y, 1e-4f);
    EXPECT_NEAR(after.z, before.z, 1e-4f);
}

TEST(AttitudeEstimator, HighAccelSkipsUpdate) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.0f, 0.0f, 0.0f};
    CoreMath::Vector3 accel_valid{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 100; i++) {
        estimator.update(gyro, accel_valid, 0.01f);
    }

    CoreMath::Quaternion before = estimator.getAttitude();

    CoreMath::Vector3 accel_spike{0.0f, 0.0f, 3.0f};
    estimator.update(gyro, accel_spike, 0.01f);

    CoreMath::Quaternion after = estimator.getAttitude();

    EXPECT_NEAR(after.w, before.w, 1e-4f);
    EXPECT_NEAR(after.x, before.x, 1e-4f);
    EXPECT_NEAR(after.y, before.y, 1e-4f);
    EXPECT_NEAR(after.z, before.z, 1e-4f);
}

TEST(AttitudeEstimator, LongRunNoDrift) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.0f, 0.0f, 0.0f};
    CoreMath::Vector3 accel{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 60000; i++) {
        estimator.update(gyro, accel, 0.001f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();

    EXPECT_NEAR(euler.x, 0.0f, 0.01f);
    EXPECT_NEAR(euler.y, 0.0f, 0.01f);
}

TEST(AttitudeEstimator, GyroThenAccelCorrects) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.5f, 0.0f, 0.0f};
    CoreMath::Vector3 accel_bad{0.0f, 0.0f, 3.0f};

    for (int i = 0; i < 200; i++) {
        estimator.update(gyro, accel_bad, 0.01f);
    }

    CoreMath::Vector3 gyro_stop{0.0f, 0.0f, 0.0f};
    CoreMath::Vector3 accel_good{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 5000; i++) {
        estimator.update(gyro_stop, accel_good, 0.001f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();

    EXPECT_NEAR(euler.x, 0.0f, 0.1f);
    EXPECT_NEAR(euler.y, 0.0f, 0.1f);
}

TEST(AttitudeEstimator, CovarianceRemainsFinite) {
    AttitudeEstimator estimator;

    CoreMath::Vector3 gyro{0.3f, -0.1f, 0.2f};
    CoreMath::Vector3 accel{0.0f, 0.0f, 1.0f};

    for (int i = 0; i < 10000; i++) {
        estimator.update(gyro, accel, 0.001f);
    }

    CoreMath::Vector3 euler = estimator.getEuler();
    CoreMath::Quaternion q = estimator.getAttitude();

    EXPECT_TRUE(std::isfinite(euler.x));
    EXPECT_TRUE(std::isfinite(euler.y));
    EXPECT_TRUE(std::isfinite(euler.z));
    EXPECT_TRUE(std::isfinite(q.w));
    EXPECT_TRUE(std::isfinite(q.x));
    EXPECT_TRUE(std::isfinite(q.y));
    EXPECT_TRUE(std::isfinite(q.z));
}

TEST(AttitudeEstimator, DebugGravityReference) {
    CoreMath::Vector3 g = CoreMath::RotateVector(CoreMath::Quaternion::Identity(), CoreMath::Vector3::Up());

    EXPECT_NEAR(g.x, 0.0f, 1e-5f);
    EXPECT_NEAR(g.y, 0.0f, 1e-5f);
    EXPECT_NEAR(g.z, 1.0f, 1e-5f);
}