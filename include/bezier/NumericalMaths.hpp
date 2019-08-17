/**
 * @file    Types.hpp
 *
 * @brief   Types
 *
 * @author  btran
 *
 * @date    2019-06-19
 *
 * Bezier
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <vector>

namespace robotics
{
namespace maths
{
constexpr double FUZZY_EPSILON = 1e-4;

/**
 *  @brief function to calculate binomial coefficients
 *
 *  This is a O(k) algorithm. return type of double is used to avoid overflow
 *
 *  @param n for kCn
 *  @param k for kCn
 *  @return double
 */
inline double binomialCoeff(const size_t n, const size_t k)
{
    if (n < k) {
        throw std::out_of_range("n must not be less than k");
    }

    double res = 1.0;

    const size_t optimizedK = (k > n - k) ? n - k : k;

    for (size_t i = 0; i < optimizedK; ++i) {
        res = (n - i) * res / (i + 1);
    }

    return res;
}

std::vector<double> binomialCoeffs(const size_t n);

std::vector<double> polynomialCoeffs(const size_t n, const double t);

template <typename T>
bool combinedToleranceEquals(const T& val, const T& correctVal, const T& epsilon = std::numeric_limits<T>::epsilon())
{
    const T maxXYOne = std::max({static_cast<T>(1.0f), std::fabs(val), std::fabs(correctVal)});
    return std::fabs(val - correctVal) <= epsilon * maxXYOne;
}

template <typename T> bool isWithinZeroAndOne(const T& x, const T& epsilon = std::numeric_limits<T>::epsilon())
{
    return x >= -epsilon && x <= (static_cast<T>(1.0f) + epsilon);
}

template <typename T> constexpr int sgn(const T& a, const T& b) noexcept
{
    return (a > b) - (a < b);
}

template <typename T> constexpr int sgn(const T& a) noexcept
{
    return sgn(a, T(0));
}

}  // namespace maths
}  // namespace robotics
