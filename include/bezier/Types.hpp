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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <cstdlib>
#include <vector>

namespace robotics
{
namespace maths
{
constexpr double FUZZY_EPSILON = 1e-7;

/**
 *  @brief function to calculate binomial coefficients
 *
 *  This is a O(k) algorithm. return type of double is used to avoid overflow
 *
 *  @param param
 *  @return return type
 */

inline double binomialCoeff(size_t n, size_t k);

std::vector<double> binomialCoeffs(size_t n);

std::vector<double> polynomialCoeffs(size_t n, double t);

}  // namespace maths
}  // namespace robotics

#endif /* TYPES_HPP_ */
