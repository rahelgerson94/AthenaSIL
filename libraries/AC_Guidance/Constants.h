#pragma once

#pragma once
#include <array>
#include <cmath>
#include <limits>

#define X 0
#define Y 1
#define Z 2

#define ROLL  0
#define PITCH 1
#define YAW   2

#define R 0
#define V 1
#define A 2

#define FT2M     0.3048
#define M2FT     3.28084
#define M2INCH   39.3701
#define INCH2M   0.0254
#define RAD2DEG  (180.0 / 3.141596)
#define DEG2RAD  (3.141596 / 180.0)
#define HZ2RADS  (2.0 * 3.141596)
#define RADS2HZ  (1.0 / HZ2RADS)

#define KG2LBS   2.20462
#define LB2KG    (1.0 / KG2LBS)
#define N2LBF    0.224809
#define NM2LBF_FT 0.737562

constexpr double g = 9.81;
constexpr std::array<double, 3> gInI = {0.0, 0.0, -g};
constexpr std::array<double, 3> ZERO = {0.0, 0.0, 0.0};
constexpr std::array<double, 3> UNUSED = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()
};
constexpr std::array<double, 4> QZERO = {1.0, 0.0, 0.0, 0.0};

constexpr double dt = 0.05;
