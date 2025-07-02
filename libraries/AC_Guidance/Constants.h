#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <cstdlib> // for getenv
using std::vector;
using std::string;
using std::cout;
using std::endl;

namespace GuidanceConstants {

    // Axis indices
    constexpr int X = 0;
    constexpr int Y = 1;
    constexpr int Z = 2;

    // Euler angle indices
    constexpr int ROLL  = 0;
    constexpr int PITCH = 1;
    constexpr int YAW   = 2;

    // Kinematic state indices
    constexpr int POS   = 0;
    constexpr int VEL   = 1;
    constexpr int ACCEL = 2;

    // Unit conversions
    constexpr double FT2M   = 0.3048;
    constexpr double M2FT   = 3.28084;
    constexpr double M2INCH = 39.3701;
    constexpr double INCH2M = 0.0254;

    constexpr double PI = 3.141596;
    constexpr double RAD2DEG = 180.0 / PI;
    constexpr double DEG2RAD = PI / 180.0;
    constexpr double HZ2RADS = 2.0 * PI;
    constexpr double RADS2HZ = 1.0 / HZ2RADS;

    constexpr double KG2LBS    = 2.20462;
    constexpr double LB2KG     = 1.0 / KG2LBS;
    constexpr double N2LBF     = 0.224809;
    constexpr double NM2LBF_FT = 0.737562;

    // Gravity
    constexpr double g = 9.81;
    constexpr std::array<double, 3> gInI = {0.0, 0.0, -g};

    // Common vectors
    constexpr std::array<double, 3> ZERO = {0.0, 0.0, 0.0};
    constexpr std::array<double, 3> UNUSED = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()
    };
}
