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

#include "bezier/NumericalMaths.hpp"

namespace robotics
{
namespace maths
{
std::vector<double> binomialCoeffs(const size_t n)
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

std::vector<double> polynomialCoeffs(const size_t n, const double t)
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
