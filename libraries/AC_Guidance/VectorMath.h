#pragma once
#include <vector>
#include <stdexcept>
#include <cmath> //for isnan
#include "Constants.h"

// Vector addition: a + b
inline std::vector<double> operator+(const std::vector<double>& a, const std::vector<double>& b)
{
    if (a.size() != b.size())
        throw std::invalid_argument("Vector addition error: size mismatch");
    
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] + b[i];
    
    return result;
}

inline std::vector<double> operator-(const std::vector<double>& a, const std::vector<double>& b)
{
    if (a.size() != b.size())
        throw std::invalid_argument("Vector addition error: size mismatch");
    
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result[i] = a[i]- b[i];
    
    return result;
}

// Scalar multiplication: scalar * vector
inline std::vector<double> operator*(double scalar, const std::vector<double>& v)
{
    std::vector<double> result(v.size());
    for (size_t i = 0; i < v.size(); ++i)
        result[i] = scalar * v[i];
    
    return result;
}

// Scalar multiplication: vector * scalar
inline std::vector<double> operator*(const std::vector<double>& v, double scalar)
{
    return scalar * v;
}
//divison of a vector by a scalr
inline std::vector<double> operator/( const std::vector<double>& v, double scalar)
{
    std::vector<double> result(v.size());
    for (size_t i = 0; i < v.size(); ++i)
        result[i] =  v[i]/scalar;
    
    return result;
}

inline double dot(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != 3 || b.size() != 3)
        throw std::invalid_argument("dotProduct: vectors must be 3D");
    return a[X]*b[X] + a[Y]*b[Y] + a[Z]*b[Z];
}

// Compute the cross product of two 3D vectors
inline std::vector<double> cross(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != 3 || b.size() != 3)
        throw std::invalid_argument("crossProduct: vectors must be 3D");
    return 
    {
        a[Y]*b[Z] - a[Z]*b[Y],
        a[Z]*b[X] - a[X]*b[Z],
        a[X]*b[Y] - a[Y]*b[X]
    };
}
inline double norm(const std::vector<double>& v) {
    double sumSq = 0.0;
    //printf("entering norm\n") ;
    for (const auto& x : v) 
    {
        sumSq += x * x;
    }
    //printf("exiting norm\n");
    return std::sqrt(sumSq);
    
}

inline std::vector<double> clipVec(const std::vector<double>& input, 
    const double minVal, 
    const double maxVal) 
    {
    std::vector<double> result = input;
    for (auto& val : result) 
    {
        val = std::min(std::max(val, minVal), maxVal);
    }
    return result;
}

inline bool hasNan(std::vector<double>& input)
{
    for (int i = 0; i < input.size(); ++i) 
    {
        if (std::isnan(input[i]))
        {
            return true;
        }
    }
    return false;
}

inline void printVec(std::vector<double>& input, char* name)
{
    printf("%s: [", name);
    int N = input.size();
    for (int i = 0; i < N; ++i) 
    {
        printf("%.2f  ,",input[i]);
    }
    printf("%.2f  ]\n",input[N-1]);
}
