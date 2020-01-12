#pragma once

#include <cmath>
#include <iostream>

namespace tf
{

// ************************************************************************************************
// * Vector3                                                                                      *
// ************************************************************************************************

/** Vector3 data structure */
struct Vector3
{
    // members
    double x = 0;
    double y = 0;
    double z = 0;

    // methods
    constexpr Vector3() = default;
    constexpr Vector3(double x, double y, double z)
      : x(x)
      , y(y)
      , z(z)
    {}
};

/** Adds two vectors */
constexpr Vector3 operator+(const Vector3& a, const Vector3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

/** Subtracts two vectors */
constexpr Vector3 operator-(const Vector3& a, const Vector3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

/** Dot product */
constexpr double operator*(const Vector3& a, const Vector3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** Vector negation */
constexpr Vector3 operator-(const Vector3& v)
{
    return {-v.x, -v.y, -v.z};
}

/** Streaming helper */
std::ostream& operator<<(std::ostream& os, const Vector3& v)
{
    return os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
}

// ************************************************************************************************
// * Quaternion                                                                                   *
// ************************************************************************************************

/** Quaternion data structure */
struct Quaternion
{
    // members
    double w = 1;
    Vector3 v;

    // methods
    constexpr Quaternion() = default;
    constexpr Quaternion(double w, const Vector3& v)
      : w(w)
      , v(v)
    {}
    constexpr Quaternion(double w, double x, double y, double z)
      : w(w)
      , v{x,y,z}
    {}
    constexpr Quaternion(const Vector3& axis, double angle)
    {
        const double c = std::cos(angle/2);
        const double s = std::sin(angle/2);
        w = c;
        v = {s * axis.x, s * axis.y, s * axis.z};
    }
    double norm() const
    {
        return std::sqrt(w * w + v * v);
    }
};

/** Quaternion conjugate */
constexpr Quaternion operator~(const Quaternion& q)
{
    return {q.w, -q.v};
}

/** Combine two rotations */
constexpr Quaternion operator*(const Quaternion& a, const Quaternion& b)
{
    // Hamilton product
    return {
        -a.v.x * b.v.x - a.v.y * b.v.y - a.v.z * b.v.z + a.w * b.w,
        {
             a.v.x * b.w + a.v.y * b.v.z - a.v.z * b.v.y + a.w * b.v.x,
            -a.v.x * b.v.z + a.v.y * b.w + a.v.z * b.v.x + a.w * b.v.y,
             a.v.x * b.v.y - a.v.y * b.v.x + a.v.z * b.w + a.w * b.v.z,
        }
    };
}

/** Rotates a vector */
constexpr Vector3 operator*(const Quaternion& q, const Vector3& v)
{
    Quaternion p{0, v};  // vector represented as pure quaternion
    p = q * p * ~q;
    return p.v;
}

/** Streaming helper */
std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
    return os << "[" << q.w << ", " << q.v << "]";
}

// ************************************************************************************************
// * Transform                                                                                    *
// ************************************************************************************************

/** Transforms contain a position and a rotation */
struct Transform
{
    // members
    Vector3 position;
    Quaternion rotation;

    // methods
    constexpr Transform() = default;
    constexpr Transform(const Vector3& position, const Quaternion& rotation)
      : position(position)
      , rotation(rotation)
    {}
};

/** Inverse transformation */
constexpr Transform operator~(const Transform& tf)
{
    return {~tf.rotation * -tf.position, ~tf.rotation};
}

/** Combine two transformations */
constexpr Transform operator*(const Transform& a, const Transform& b)
{
    return {a.position + a.rotation * b.position, a.rotation * b.rotation};
}

/** Transform a vector */
constexpr Vector3 operator*(const Transform& tf, const Vector3& v)
{
    return {tf.position + tf.rotation * v};
}

/** Streaming helper */
std::ostream& operator<<(std::ostream& os, const Transform& tf)
{
    return os << "{pos: " << tf.position << ", rot: " << tf.rotation << "}";
}

}  // namespace tf
