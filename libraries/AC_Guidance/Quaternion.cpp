#include "Quaternion.h"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cassert>
namespace GC = GuidanceConstants;

Vec4 GuidanceUtils::Quaternion::quaternionFromDcm(const Mat3& dcm) {
    double m00 = dcm[GC::X][GC::X], m01 = dcm[GC::X][GC::Y], m02 = dcm[GC::X][GC::Z];
    double m10 = dcm[GC::Y][GC::X], m11 = dcm[GC::Y][GC::Y], m12 = dcm[GC::Y][GC::Z];
    double m20 = dcm[GC::Z][GC::X], m21 = dcm[GC::Z][GC::Y], m22 = dcm[GC::Z][GC::Z];
    double w, x, y, z;

    double trace = m00 + m11 + m22;
    if (trace > 0) {
        double s = 2.0 * std::sqrt(trace + 1.0);
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    } else if (m11 > m22) {
        double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }
    Vec4 q = { w, x, y, z };
    double n = norm(q);
    for (auto& qi : q) qi /= n;
    return q;
}

Vec4 GuidanceUtils::Quaternion::quaternionFromEulerAngles321(const Vec3& e) {
    double phi = e[GC::X], theta = e[GC::Y], psi = e[GC::Z];
    double cPsi = std::cos(psi * 0.5);
    double sPsi = std::sin(psi * 0.5);
    double cTheta = std::cos(theta * 0.5);
    double sTheta = -std::sin(theta * 0.5);
    double cPhi = std::cos(phi * 0.5);
    double sPhi = std::sin(phi * 0.5);

    double q0 = cPhi * cTheta * cPsi + sPhi * sTheta * sPsi;
    double q1 = cPhi * sTheta * sPsi - sPhi * cTheta * cPsi;
    double q2 = -cPhi * sTheta * cPsi - sPhi * cTheta * sPsi;
    double q3 = sPhi * sTheta * cPsi - cPhi * cTheta * sPsi;

    Vec4 q = { q0, q1, q2, q3 };
    q = q/norm(q);
    return q;
}

Vec4 GuidanceUtils::Quaternion::multiply(const Vec4& q, const Vec4& p) {
    double q0 = q[GC::X], q1 = q[GC::Y], q2 = q[GC::Z], q3 = q[3];
    double p0 = p[GC::X], p1 = p[GC::Y], p2 = p[GC::Z], p3 = p[3];
    return {
        q0*p0 - q1*p1 - q2*p2 - q3*p3,
        q0*p1 + q1*p0 + q2*p3 - q3*p2,
        q0*p2 - q1*p3 + q2*p0 + q3*p1,
        q0*p3 + q1*p2 - q2*p1 + q3*p0
    };
}

Vec4 GuidanceUtils::Quaternion::conjugate(const Vec4& q) {
    return { q[GC::X], -q[GC::Y], -q[GC::Z], -q[3] };
}

Vec3 GuidanceUtils::Quaternion::rotateVectorByQuaternion(const Vec4& q, const Vec3& v) {
    Vec4 vQuat = {0, v[GC::X], v[GC::Y], v[GC::Z]};
    Vec4 qNorm = q;
    double n = norm(qNorm);
    for (auto& qi : qNorm) qi /= n;
    Vec4 qConj = GuidanceUtils::Quaternion::conjugate(qNorm);
    Vec4 qv = GuidanceUtils::Quaternion::multiply(qNorm, vQuat);
    Vec4 rotatedQuat = GuidanceUtils::Quaternion::multiply(qv, qConj);
    return {rotatedQuat[GC::Y], rotatedQuat[GC::Z], rotatedQuat[3]};
}

Mat3 GuidanceUtils::Quaternion::dcmFromEulerAngles32(const Vec3& a) {
    double phi = a[GC::X], theta = a[GC::Y], psi = a[GC::Z];
    double c_psi = cos(psi), s_psi = sin(psi);
    double c_theta = cos(theta), s_theta = sin(theta);
    return {{
        {c_psi * c_theta, s_psi * c_theta, s_theta},
        {-s_psi, c_psi, 0},
        {-c_psi * s_theta, -s_psi * s_theta, c_theta}
    }};
}

Mat3 GuidanceUtils::Quaternion::dcmFromEulerAngles321(const Vec3& a) {
    double phi = a[GC::X], theta = a[GC::Y], psi = a[GC::Z];
    double cphi = cos(phi), sphi = sin(phi);
    double ctheta = cos(theta), stheta = sin(theta);
    double cpsi = cos(psi), spsi = sin(psi);
    return {{
        {cpsi * ctheta, spsi * ctheta, stheta},
        {-cpsi * stheta * sphi - spsi * cphi, -spsi * stheta * sphi + cpsi * cphi, ctheta * sphi},
        {-cpsi * stheta * cphi + spsi * sphi, -spsi * stheta * cphi - cpsi * sphi, ctheta * cphi}
    }};
}

Vec3 GuidanceUtils::Quaternion::eulerAngles321FromQuaternion(const Vec4& q_) {
    Vec4 q = GuidanceUtils::Quaternion::conjugate(q_);
    double q0 = q[GC::X], q1 = q[GC::Y], q2 = q[GC::Z], q3 = q[3];
    double q0_2 = q0*q0, q1_2 = q1*q1, q2_2 = q2*q2, q3_2 = q3*q3;
    double phi = atan2(2*(q2*q3 + q0*q1), q0_2 - q1_2 - q2_2 + q3_2);
    double theta = asin(2*(q1*q3 - q0*q2));
    double psi = atan2(2*(q1*q2 + q0*q3), q0_2 + q1_2 - q2_2 - q3_2);
    return {phi, theta, psi};
}

Vec3 GuidanceUtils::Quaternion::eulerAngles321FromQuaternion2(const Vec4& q) {
    double q0 = q[GC::X], q1 = q[GC::Y], q2 = q[GC::Z], q3 = q[3];
    double q0_2 = q0*q0, q1_2 = q1*q1, q2_2 = q2*q2, q3_2 = q3*q3;
    double phi = atan2(2*(q2*q3 - q0*q1), q0_2 - q1_2 - q2_2 + q3_2);
    double theta = asin(2*(q1*q3 - q0*q2));
    double psi = atan2(2*(q1*q2 - q0*q3), q0_2 + q1_2 - q2_2 - q3_2);
    return {phi, theta, psi};
}

Vec4 GuidanceUtils::Quaternion::computeDerivative(const Vec4& q, const Vec3& w) {
    Vec4 wQuat = GuidanceUtils::Quaternion::getPureQuaternion(w);
    Vec4 dq = 0.5* GuidanceUtils::Quaternion::multiply(q, wQuat);
    return dq;
}

Vec4 GuidanceUtils::Quaternion::getPureQuaternion(const Vec3& v) {
    return {0, v[GC::X], v[GC::Y], v[GC::Z]};
}

Vec3 GuidanceUtils::Quaternion::eulerAngles321FromDcm(const Mat3& dcm) {
    double theta = asin(dcm[GC::X][GC::Z]);
    double psi = atan2(dcm[GC::X][GC::Y], dcm[GC::X][GC::X]);
    double phi = atan2(dcm[GC::Y][GC::Z], dcm[GC::Z][GC::Z]);
    return {phi, theta, psi};
}

Mat3 GuidanceUtils::Quaternion::quat2Dcm(const Vec4& q) {
    double q0 = q[GC::X], q1 = q[GC::Y], q2 = q[GC::Z], q3 = q[3];
    return {{
        {1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)},
        {2*(q1*q2 + q0*q3), 1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)},
        {2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2)}
    }};
}
