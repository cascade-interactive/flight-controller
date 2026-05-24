/*
 * CoreMath.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#pragma once

#include <cmath>

namespace CoreMath {

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
        if (sq_size > 0.000001f) {
            float inv_size = 1.0f / std::sqrt(sq_size);
            x *= inv_size;
            y *= inv_size;
            z *= inv_size;
        }
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
};
} // namespace CoreMath
