#include "gtest/gtest.h"

#include "include/transform.hpp"

using namespace tf;

constexpr double EPS = 1e-10;
::testing::AssertionResult VecEqual(const Vector3& a, const Vector3& b)
{
    auto almostEqual = [](double p, double q) { return std::abs(p - q) < EPS; };
    if (almostEqual(a.x, b.x) && almostEqual(a.y, b.y) && almostEqual (a.z, b.z))
    {
        return ::testing::AssertionSuccess() << a << " == " << b;
    }
    else
    {
        return ::testing::AssertionFailure() << a << " != " << b;
    }
}

// ************************************************************************************************
// * Vector3                                                                                      *
// ************************************************************************************************
TEST(Vector3, Ctor) {
    {
    const Vector3 v;
    ASSERT_TRUE(VecEqual(v, {0,0,0}));
    }
    {
    const Vector3 v{1,2,3};
    ASSERT_TRUE(VecEqual(v, {1,2,3}));
    }
}

TEST(Vector3, Addition) {
    const Vector3 a{1,2,3};
    const Vector3 b{4,5,6};
    const auto c = a + b;
    ASSERT_TRUE(VecEqual(c, {5,7,9}));
}

TEST(Vector3, Subtraction) {
    const Vector3 a{1,2,3};
    const Vector3 b{2,2,1};
    const auto c = a - b;
    ASSERT_TRUE(VecEqual(c, {-1,0,2}));
}

TEST(Vector3, DotProduct) {
    const Vector3 a{1,2,3};
    const Vector3 b{2,-2,1};
    const auto c = a * b;
    ASSERT_EQ(c, 1);
}

TEST(Vector3, Negation) {
    const Vector3 a{1,2,-3};
    const auto b = -a;
    ASSERT_TRUE(VecEqual(b, {-1,-2,3}));
}

// ************************************************************************************************
// * Quaternion                                                                                   *
// ************************************************************************************************
TEST(Quaternion, Ctor) {
    {
    const Quaternion q;
    ASSERT_EQ(q.w, 1);
    ASSERT_TRUE(VecEqual(q.v, {0,0,0}));
    }
    {
    const Quaternion q{0.1, {2,3,4}};
    ASSERT_EQ(q.w, 0.1);
    ASSERT_TRUE(VecEqual(q.v, {2,3,4}));
    }
    {
    const Quaternion q{{0,0,1}, M_PI};
    ASSERT_NEAR(q.w, 0, 1e-10);
    ASSERT_TRUE(VecEqual(q.v, {0,0,1}));
    }
}

TEST(Quaternion, Conjugation) {
    const Quaternion q1{1, {2,3,4}};
    const Quaternion q2 = ~q1;
    ASSERT_EQ(q2.w, 1);
    ASSERT_TRUE(VecEqual(q2.v, {-2,-3,-4}));
}

// ************************************************************************************************
// * Vector / Quaternion                                                                          *
// ************************************************************************************************
TEST(VectorQuaternion, Rotate) {
    const Quaternion q{{0,0,1}, M_PI/2};
    const Vector3 v{1,2,3};
    const Vector3 w = q * v;
    ASSERT_TRUE(VecEqual(w, {-2,1,3}));
}

// ************************************************************************************************
// * Transform                                                                                    *
// ************************************************************************************************
TEST(Transform, Invert) {
    const Transform tf1{{1,2,3}, {{0,0,1}, M_PI/2}};
    const Transform tf2 = ~tf1;
    ASSERT_TRUE(VecEqual(tf2.position, {-2,1,-3}));
    ASSERT_NEAR(tf2.rotation.w, 0.5 * std::sqrt(2), EPS);
    ASSERT_TRUE(VecEqual(tf2.rotation.v, {0,0,-0.5 * std::sqrt(2)}));
    ASSERT_EQ(tf2.rotation.v.x, 0);
    ASSERT_EQ(tf2.rotation.v.y, 0);
}

// ************************************************************************************************
// * Transform / Vector3                                                                          *
// ************************************************************************************************
TEST(TransformVector, Transform) {
    const Transform tf{{1,2,3}, {{0,0,1}, M_PI/2}};
    const Vector3 v{4,5,6};
    const Vector3 w = tf * v;
    ASSERT_TRUE(VecEqual(w, {-4,6,9}));
}

TEST(TransformVector, Transform2) {
    // Transform looking at the world from (0,0), under a 30 degree rotation
    const Transform tf{{0,0,0}, {{0,0,1}, M_PI/6}};
    // So this unit vector is dead ahead
    const Vector3 v{std::cos(M_PI/6),std::sin(M_PI/6),0};
    const Vector3 w = ~tf * v;
    ASSERT_TRUE(VecEqual(w, {1,0,0}));
}

TEST(TransformVector, Transform3) {
    // Transform looking at the world from (1,1), under a 45 degree rotation
    const Transform tf{{1,1,0}, {{0,0,1}, M_PI/4}};
    // So this vector is dead ahead
    const Vector3 v{2,2,0};
    const Vector3 w = ~tf * v;
    ASSERT_TRUE(VecEqual(w, {std::sqrt(2),0,0}));
}