/*
 * CoreMath.hpp
 *
 *  Created on: May 23, 2026
 *      Author: Alexander
 */

#pragma once

#include <cmath>

namespace CoreMath {

// ------------------------------
// Constants
// ------------------------------
constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = 2.0f * Pi;
constexpr float DegToRad = Pi / 180.0f;
constexpr float RadToDeg = 180.0f / Pi;
constexpr float SmallNumber = 0.000001f;

// ------------------------------
// Scalar helpers
// ------------------------------

// Parameters: degrees is an angle in degrees. Output: the same angle in radians. Works by multiplying by pi / 180.
[[nodiscard]] inline float DegreesToRadians(float degrees) {
    return degrees * DegToRad;
}

// Parameters: radians is an angle in radians. Output: the same angle in degrees. Works by multiplying by 180 / pi.
[[nodiscard]] inline float RadiansToDegrees(float radians) {
    return radians * RadToDeg;
}

// Parameters: value plus a min and max. Output: value limited to that range. Works by returning a bound when value is outside it.
[[nodiscard]] inline float Clamp(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// Parameters: a and b are endpoints, alpha is the blend amount. Output: value between a and b. Works by adding alpha of the delta to a.
[[nodiscard]] inline float Lerp(float a, float b, float alpha) {
    return a + ((b - a) * alpha);
}

// Parameters: angle_rad is any angle in radians. Output: equivalent angle from -pi to pi. Works by adding/subtracting 2pi.
[[nodiscard]] inline float WrapPi(float angle_rad) {
    while (angle_rad > Pi) {
        angle_rad -= TwoPi;
    }
    while (angle_rad < -Pi) {
        angle_rad += TwoPi;
    }
    return angle_rad;
}

// Parameters: target_rad and current_rad are angles in radians. Output: shortest signed angle error. Works by wrapping target - current.
[[nodiscard]] inline float AngleDiffRad(float target_rad, float current_rad) {
    return WrapPi(target_rad - current_rad);
}

// Parameters: cutoff_hz is filter cutoff and dt is timestep seconds. Output: first-order low-pass alpha. Works from the RC filter formula.
[[nodiscard]] inline float LowPassAlpha(float cutoff_hz, float dt) {
    float rc = 1.0f / (TwoPi * cutoff_hz);
    return dt / (rc + dt);
}

// ------------------------------
// Vector3
// ------------------------------

struct Vector3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    // Parameters: none. Output: zero vector. Works by initializing x, y, and z to 0.
    inline Vector3() : x(0.0f), y(0.0f), z(0.0f) {
    }

    // Parameters: one value for each axis. Output: vector with those components. Works by copying inputs to x, y, and z.
    inline Vector3(float in_x, float in_y, float in_z) : x(in_x), y(in_y), z(in_z) {
    }

    // Parameters: value used for all axes. Output: vector with x, y, and z all equal. Works by copying value to every component.
    inline explicit Vector3(float value) : x(value), y(value), z(value) {
    }

    // Parameters: none. Output: vector (0, 0, 0). Works by constructing a Vector3 with zero components.
    [[nodiscard]] inline static Vector3 Zero() {
        return Vector3(0.0f, 0.0f, 0.0f);
    }

    // Parameters: none. Output: vector (1, 1, 1). Works by constructing a Vector3 with one in each component.
    [[nodiscard]] inline static Vector3 One() {
        return Vector3(1.0f, 1.0f, 1.0f);
    }

    // Parameters: none. Output: up direction vector. Works by using +Z as up.
    [[nodiscard]] inline static Vector3 Up() {
        return Vector3(0.0f, 0.0f, 1.0f);
    }

    // Parameters: none. Output: forward direction vector. Works by using +X as forward.
    [[nodiscard]] inline static Vector3 Forward() {
        return Vector3(1.0f, 0.0f, 0.0f);
    }

    // Parameters: none. Output: squared vector length. Works by summing x*x, y*y, and z*z.
    [[nodiscard]] inline float SizeSquared() const {
        return (x * x) + (y * y) + (z * z);
    }

    // Parameters: none. Output: vector length. Works by taking the square root of SizeSquared().
    [[nodiscard]] inline float Size() const {
        return std::sqrt(SizeSquared());
    }

    // Parameters: none. Output: modifies this vector to unit length. Works by multiplying components by inverse length when nonzero.
    inline void Normalize() {
        float sq_size = SizeSquared();
        if (sq_size > SmallNumber) {
            float inv_size = 1.0f / std::sqrt(sq_size);
            x *= inv_size;
            y *= inv_size;
            z *= inv_size;
        }
    }

    // Parameters: none. Output: normalized copy of this vector. Works by copying this vector and normalizing the copy.
    [[nodiscard]] inline Vector3 Normalized() const {
        Vector3 result = *this;
        result.Normalize();
        return result;
    }

    // Parameters: other vector. Output: scalar dot product. Works by summing matching component products.
    [[nodiscard]] inline float Dot(const Vector3 &other) const {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    // Parameters: other vector. Output: cross product vector. Works by returning the vector perpendicular to both inputs.
    [[nodiscard]] inline Vector3 Cross(const Vector3 &other) const {
        return Vector3((y * other.z) - (z * other.y), (z * other.x) - (x * other.z), (x * other.y) - (y * other.x));
    }

    // Parameters: tolerance is the near-zero threshold. Output: true if length is within tolerance. Works using squared length.
    [[nodiscard]] inline bool IsNearlyZero(float tolerance = SmallNumber) const {
        return SizeSquared() <= (tolerance * tolerance);
    }

    // Parameters: other vector. Output: component-wise sum. Works by adding matching components.
    inline Vector3 operator+(const Vector3 &other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    // Parameters: other vector. Output: component-wise difference. Works by subtracting matching components.
    inline Vector3 operator-(const Vector3 &other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    // Parameters: scalar multiplier. Output: scaled vector. Works by multiplying each component by scalar.
    inline Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    // Parameters: scalar divisor. Output: scaled vector. Works by dividing each component by scalar.
    inline Vector3 operator/(float scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    // Parameters: other vector. Output: reference to this vector after addition. Works by adding other into each component.
    inline Vector3 &operator+=(const Vector3 &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    // Parameters: other vector. Output: reference to this vector after subtraction. Works by subtracting other from each component.
    inline Vector3 &operator-=(const Vector3 &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    // Parameters: scalar multiplier. Output: reference to this vector after scaling. Works by multiplying each component in place.
    inline Vector3 &operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
};

// Parameters: scalar and vector. Output: scaled vector. Works by forwarding to vector * scalar.
[[nodiscard]] inline Vector3 operator*(float scalar, const Vector3 &vector) {
    return vector * scalar;
}

// Parameters: degrees vector. Output: radians vector. Works by multiplying every component by pi / 180.
[[nodiscard]] inline Vector3 DegreesToRadians(const Vector3 &degrees) {
    return degrees * DegToRad;
}

// Parameters: radians vector. Output: degrees vector. Works by multiplying every component by 180 / pi.
[[nodiscard]] inline Vector3 RadiansToDegrees(const Vector3 &radians) {
    return radians * RadToDeg;
}

// ------------------------------
// "The Matrix"
// ------------------------------

// TODO: Add comments
struct Matrix3 {
    float m[3][3]{};

    inline Matrix3() : m{} {
    }

    [[nodiscard]] inline static Matrix3 Identity() {
        Matrix3 result;
        result.m[0][0] = 1.0f;
        result.m[1][1] = 1.0f;
        result.m[2][2] = 1.0f;
        return result;
    }

    [[nodiscard]] inline static Matrix3 Zero() {
        return Matrix3();
    }

    [[nodiscard]] inline Matrix3 operator*(const Matrix3 &other) const {
        Matrix3 result;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    result.m[i][j] += m[i][k] * other.m[k][j];
        return result;
    }

    [[nodiscard]] inline Vector3 operator*(const Vector3 &v) const {
        return Vector3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                       m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                       m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
    }

    [[nodiscard]] inline Matrix3 Transpose() const {
        Matrix3 result;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                result.m[i][j] = m[j][i];
        return result;
    }
};

// ------------------------------
// Quaternion
// ------------------------------

struct Quaternion {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    // Parameters: none. Output: identity rotation quaternion. Works by setting w=1 and vector part to zero.
    inline Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {
    }

    // Parameters: one value for each quaternion component. Output: quaternion with those components. Works by copying inputs.
    inline Quaternion(float in_w, float in_x, float in_y, float in_z) : w(in_w), x(in_x), y(in_y), z(in_z) {
    }

    // Parameters: none. Output: identity quaternion. Works by returning (1, 0, 0, 0).
    [[nodiscard]] inline static Quaternion Identity() {
        return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    }

    // Parameters: gyro_rad_s is angular velocity in radians/sec. Output: pure quaternion. Works by putting gyro in x/y/z and zero in w.
    [[nodiscard]] inline static Quaternion FromAngularVelocity(const Vector3 &gyro_rad_s) {
        return Quaternion(0.0f, gyro_rad_s.x, gyro_rad_s.y, gyro_rad_s.z);
    }

    // Parameters: none. Output: squared quaternion length. Works by summing w*w, x*x, y*y, and z*z.
    [[nodiscard]] inline float SizeSquared() const {
        return (w * w) + (x * x) + (y * y) + (z * z);
    }

    // Parameters: none. Output: quaternion length. Works by taking the square root of SizeSquared().
    [[nodiscard]] inline float Size() const {
        return std::sqrt(SizeSquared());
    }

    // Parameters: none. Output: modifies this quaternion to unit length. Works by inverse-length scaling, or identity if too small.
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

    // Parameters: none. Output: normalized copy of this quaternion. Works by copying this quaternion and normalizing the copy.
    [[nodiscard]] inline Quaternion Normalized() const {
        Quaternion result = *this;
        result.Normalize();
        return result;
    }

    // Parameters: none. Output: conjugate quaternion. Works by keeping w and negating x, y, and z.
    [[nodiscard]] inline Quaternion Conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Parameters: none. Output: inverse quaternion. Works by returning conjugate divided by squared length, or identity if too small.
    [[nodiscard]] inline Quaternion Inverse() const {
        float sq_size = SizeSquared();

        if (sq_size > SmallNumber) {
            return Conjugate() * (1.0f / sq_size);
        }

        return Identity();
    }

    // Parameters: other quaternion. Output: component-wise sum. Works by adding matching components.
    inline Quaternion operator+(const Quaternion &other) const {
        return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    // Parameters: other quaternion. Output: component-wise difference. Works by subtracting matching components.
    inline Quaternion operator-(const Quaternion &other) const {
        return Quaternion(w - other.w, x - other.x, y - other.y, z - other.z);
    }

    // Parameters: scalar multiplier. Output: scaled quaternion. Works by multiplying every component by scalar.
    inline Quaternion operator*(float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    // Parameters: other quaternion. Output: composed rotation/product. Works using Hamilton product multiplication.
    inline Quaternion operator*(const Quaternion &other) const {
        return Quaternion((w * other.w) - (x * other.x) - (y * other.y) - (z * other.z),
                          (w * other.x) + (x * other.w) + (y * other.z) - (z * other.y),
                          (w * other.y) - (x * other.z) + (y * other.w) + (z * other.x),
                          (w * other.z) + (x * other.y) - (y * other.x) + (z * other.w));
    }
};

[[nodiscard]] inline Vector3 QuaternionToEuler(const Quaternion &q) {
    Vector3 euler;

    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.x = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);

    sinp = Clamp(sinp, -1.0f, 1.0f);

    euler.y = std::asin(sinp);

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.z = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}
} // namespace CoreMath
