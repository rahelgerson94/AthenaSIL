#pragma once
#include <vector>
#include <array>
#include <string>
#include "VectorMath.h"
#include "Constants.h"
using std::vector;
// Convenience aliases for fixed-size vectors/matrices
using Vec3 = vector<double>;
using Vec4 = vector<double>;
using Mat3 =vector<vector<double>>;

class Quaternion 
{
public:
    // Converts a direction cosine matrix to a quaternion (scalar-first)
    static Vec4 quaternionFromDcm(const Mat3& dcm);

    // Converts ZYX Euler angles (yaw-pitch-roll) to quaternion
    static Vec4 quaternionFromEulerAngles321(const Vec3& eulerAngles321);

    // Quaternion multiplication (Hamiltonian convention)
    static Vec4 multiply(const Vec4& q, const Vec4& p);

    // Conjugate of a quaternion
    static Vec4 conjugate(const Vec4& q);

    // Rotates a vector by a quaternion (or by one derived from a DCM)
    static Vec3 rotateVectorByQuaternion(const Vec4& qOrDcm, const Vec3& v);

    // Constructs a DCM from 3-2 Euler angles (psi, theta, phi)
    static Mat3 dcmFromEulerAngles32(const Vec3& angles32);

    // Constructs a DCM from 3-2-1 Euler angles (yaw-pitch-roll)
    static Mat3 dcmFromEulerAngles321(const Vec3& angles321);

    // Converts quaternion to Euler angles (ZYX convention), version 1
    static Vec3 eulerAngles321FromQuaternion(const Vec4& q);

    // Converts quaternion to Euler angles (ZYX convention), version 2
    static Vec3 eulerAngles321FromQuaternion2(const Vec4& q);

    // Time derivative of quaternion given angular velocity vector w
    static Vec4 computeDerivative(const Vec4& q, const Vec3& w);

    // Returns a pure quaternion (0, v.x, v.y, v.z)
    static Vec4 getPureQuaternion(const Vec3& v);

    // Extracts Euler angles (ZYX) from a direction cosine matrix
    static Vec3 eulerAngles321FromDcm(const Mat3& dcm);

    // Converts a quaternion to a DCM
    static Mat3 quat2Dcm(const Vec4& q);
};


