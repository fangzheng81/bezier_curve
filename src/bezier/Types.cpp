/**
 * @file    Types.cpp
 *
 * @brief   Implementation of Types
 *
 * @author  btran
 *
 * @date    2019-06-19
 *
 * Copyright (c) organization
 *
 */

#include "bezier/Bezier.hpp"
#include <iostream>

namespace robotics
{
namespace maths
{
inline double binomialCoeff(size_t n, size_t k)
{
    if (n < k) {
        throw std::out_of_range("n must not be less than k");
    }

    double res = 1.0;

    if (k > n - k)
        k = n - k;

    for (size_t i = 0; i < k; ++i) {
        res = (n - i) * res / (i + 1);
    }

    return res;
}

std::vector<double> binomialCoeffs(size_t n)
{
    std::vector<double> results(n + 1, 0);
    results.reserve(n + 1);

    size_t k = 0;
    size_t center = n / 2;

    for (; k <= center; ++k) {
        results[k] = binomialCoeff(n, k);
        results[n - k] = results[k];
    }

    return results;
}

std::vector<double> polynomialCoeffs(size_t n, double t)
{
    std::vector<double> results;
    results.reserve(n + 1);

    for (size_t i = 0; i <= n; ++i) {
        results.emplace_back(pow(1 - t, n - i) * pow(t, i));
    }

    return results;
}

}  // namespace maths
}  // namespace robotics
