/*
 * CoreMath.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#pragma once

#include <cmath>

namespace CoreMath {

// Constants
constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float DegToRad = Pi / 180.0f;
constexpr float RadToDeg = 180.0f / Pi;
constexpr float SmallNumber = 0.000001f;

[[nodiscard]] inline float Clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

[[nodiscard]] inline float Lerp(float a, float b, float alpha) {
    return a + ((b - a) * alpha);
}

[[nodiscard]] inline float WrapPi(float angle_rad) {
    while (angle_rad > Pi) {
        angle_rad -= TwoPi;
    }
    while (angle_rad < -Pi) {
        angle_rad += TwoPi;
    }
    return angle_rad;
}

[[nodiscard]] inline float AngleDiffRad(float target_rad, float current_rad) {
    return WrapPi(target_rad - current_rad);
}

[[nodiscard]] inline float LowPassAlpha(float cutoff_hz, float dt) {
    float rc = 1.0f / (TwoPi * cutoff_hz);
    return dt / (rc + dt);
}

// Vector3s
struct Vector3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    inline Vector3() : x(0.0f), y(0.0f), z(0.0f) {
    }
    inline Vector3(float in_x, float in_y, float in_z) : x(in_x), y(in_y), z(in_z) {
    }
    inline explicit Vector3(float value) : x(value), y(value), z(value) {
    }

    [[nodiscard]] inline static Vector3 Zero() {
        return Vector3(0.0f, 0.0f, 0.0f);
    }
    [[nodiscard]] inline static Vector3 One() {
        return Vector3(1.0f, 1.0f, 1.0f);
    }
    [[nodiscard]] inline static Vector3 Up() {
        return Vector3(0.0f, 0.0f, 1.0f);
    }
    [[nodiscard]] inline static Vector3 Forward() {
        return Vector3(1.0f, 0.0f, 0.0f);
    }

    [[nodiscard]] inline float SizeSquared() const {
        return (x * x) + (y * y) + (z * z);
    }

    [[nodiscard]] inline float Size() const {
        return std::sqrt(SizeSquared());
    }

    inline void Normalize() {
        float sq_size = SizeSquared();
        if (sq_size > SmallNumber) {
            float inv_size = 1.0f / std::sqrt(sq_size);
            x *= inv_size;
            y *= inv_size;
            z *= inv_size;
        }
    }

    [[nodiscard]] inline Vector3 Normalized() const {
        Vector3 result = *this;
        result.Normalize();
        return result;
    }

    [[nodiscard]] inline float Dot(const Vector3 &other) const {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    [[nodiscard]] inline Vector3 Cross(const Vector3 &other) const {
        return Vector3((y * other.z) - (z * other.y), (z * other.x) - (x * other.z), (x * other.y) - (y * other.x));
    }

    [[nodiscard]] inline bool IsNearlyZero(float tolerance = SmallNumber) const {
        return SizeSquared() <= (tolerance * tolerance);
    }

    // Operators
    inline Vector3 operator+(const Vector3 &other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    inline Vector3 operator-(const Vector3 &other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    inline Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    inline Vector3 operator/(float scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    inline Vector3 &operator+=(const Vector3 &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    inline Vector3 &operator-=(const Vector3 &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    inline Vector3 &operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
};

// Quaternions
struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    inline Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {
    }

    inline Quaternion(float in_w, float in_x, float in_y, float in_z) : w(in_w), x(in_x), y(in_y), z(in_z) {
    }

    [[nodiscard]] inline static Quaternion Identity() {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    [[nodiscard]] inline static Quaternion FromAngularVelocity(const Vector3 &gyro_rad_s) {
        return Quaternion(0.0f, gyro_rad_s.x, gyro_rad_s.y, gyro_rad_s.z);
    }

    [[nodiscard]] inline float SizeSquared() const {
        return (w * w) + (x * x) + (y * y) + (z * z);
    }

    [[nodiscard]] inline float Size() const {
        return std::sqrt(SizeSquared());
    }

    inline void Normalize() {
        float sq_size = SizeSquared();

        if (sq_size > SmallNumber) {
            float inv_size = 1.0f / std::sqrt(sq_size);
            w *= inv_size;
            x *= inv_size;
            y *= inv_size;
            z *= inv_size;
        } else {
            *this = Identity();
        }
    }

    [[nodiscard]] inline Quaternion Normalized() const {
        Quaternion result = *this;
        result.Normalize();
        return result;
    }

    [[nodiscard]] inline Quaternion Conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    [[nodiscard]] inline Quaternion Inverse() const {
        float sq_size = SizeSquared();

        if (sq_size > SmallNumber) {
            return Conjugate() * (1.0f / sq_size);
        }

        return Identity();
    }

    // Operators
    inline Quaternion operator+(const Quaternion &other) const {
        return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    inline Quaternion operator-(const Quaternion &other) const {
        return Quaternion(w - other.w, x - other.x, y - other.y, z - other.z);
    }

    inline Quaternion operator*(float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    inline Quaternion operator*(const Quaternion &other) const {
        return Quaternion((w * other.w) - (x * other.x) - (y * other.y) - (z * other.z),
                          (w * other.x) + (x * other.w) + (y * other.z) - (z * other.y),
                          (w * other.y) - (x * other.z) + (y * other.w) + (z * other.x),
                          (w * other.z) + (x * other.y) - (y * other.x) + (z * other.w));
    }
};
} // namespace CoreMath
