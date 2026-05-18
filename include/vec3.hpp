#pragma once
#include <cmath>

namespace SoftToss
{

    struct Vec3
    {
        float x, y, z;

        constexpr Vec3() noexcept : x(0), y(0), z(0) {}
        constexpr Vec3(float x_, float y_, float z_) noexcept : x(x_), y(y_), z(z_) {}

        // arithmetic
        constexpr Vec3 operator+(const Vec3 &r) const noexcept { return {x + r.x, y + r.y, z + r.z}; }
        constexpr Vec3 operator-(const Vec3 &r) const noexcept { return {x - r.x, y - r.y, z - r.z}; }
        constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }
        constexpr Vec3 operator*(float s) const noexcept { return {x * s, y * s, z * s}; }
        constexpr Vec3 operator/(float s) const noexcept { return {x / s, y / s, z / s}; }

        constexpr Vec3 &operator+=(const Vec3 &r) noexcept
        {
            x += r.x;
            y += r.y;
            z += r.z;
            return *this;
        }
        constexpr Vec3 &operator-=(const Vec3 &r) noexcept
        {
            x -= r.x;
            y -= r.y;
            z -= r.z;
            return *this;
        }
        constexpr Vec3 &operator*=(float s) noexcept
        {
            x *= s;
            y *= s;
            z *= s;
            return *this;
        }
        constexpr Vec3 &operator/=(float s) noexcept
        {
            x /= s;
            y /= s;
            z /= s;
            return *this;
        }

        // magnitude and normalization
        float mag2() const noexcept { return x * x + y * y + z * z; }
        float mag() const noexcept { return std::sqrt(mag2()); }
        float xyMag2() const noexcept { return x * x + y * y; }
        float xyMag() const noexcept { return std::sqrt(xyMag2()); }
        Vec3 normalized() const noexcept
        {
            float len = mag();
            return (len > 1e-8f) ? (*this) / len : *this;
        }
        Vec3 &norm() noexcept
        {
            float len = mag();
            if (len > 1e-8f)
            {
                *this /= len;
            }
            return *this;
        }
    };

    inline constexpr Vec3 operator*(float s, const Vec3 &v) noexcept { return v * s; }
    inline constexpr float dot(const Vec3 &a, const Vec3 &b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline constexpr Vec3 cross(const Vec3 &a, const Vec3 &b) noexcept
    {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

} // namespace SoftToss
