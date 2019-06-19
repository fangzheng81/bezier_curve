/**
 * @file    Bezier.hpp
 *
 * @brief   Bezier
 *
 * @author  btran
 *
 * @date    2019-06-19
 *
 * Copyright (c) organization
 *
 */

#ifndef BEZIER_HPP_
#define BEZIER_HPP_

#include "Types.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

namespace robotics
{
template <typename T, size_t POINT_DIMENSION> using DefaultVector = Eigen::Matrix<T, POINT_DIMENSION, 1>;

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType = DefaultVector<T, POINT_DIMENSION>>
class Bezier
{
 public:
    static const std::vector<double> BINOMIAL_COEFFS;
    static constexpr int degree = DEGREE;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using Ptr = std::shared_ptr<Bezier>;

    Bezier();
    explicit Bezier(const VecPointType& controlPoints);
    const VecPointType& controlPoints() const;
    VecPointType& controlPoints();

    double operator()(size_t axis, double t);
    PointType operator()(double t);

 private:
    VecPointType initializePointV();

 private:
    VecPointType _controlPoints;
};

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
const std::vector<double>
    Bezier<DEGREE, T, POINT_DIMENSION, PointType>::BINOMIAL_COEFFS = maths::binomialCoeffs(DEGREE);

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
typename Bezier<DEGREE, T, POINT_DIMENSION, PointType>::VecPointType
Bezier<DEGREE, T, POINT_DIMENSION, PointType>::initializePointV()
{
    std::vector<double> v(DEGREE + 1, 0);
    PointType defaultP(v.data());
    VecPointType pointV(DEGREE + 1, defaultP);

    return pointV;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
Bezier<DEGREE, T, POINT_DIMENSION, PointType>::Bezier() : _controlPoints(initializePointV())
{
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
Bezier<DEGREE, T, POINT_DIMENSION, PointType>::Bezier(const VecPointType& controlPoints) : _controlPoints(controlPoints)
{
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
const typename Bezier<DEGREE, T, POINT_DIMENSION, PointType>::VecPointType&
Bezier<DEGREE, T, POINT_DIMENSION, PointType>::controlPoints() const
{
    return this->_controlPoints;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
typename Bezier<DEGREE, T, POINT_DIMENSION, PointType>::VecPointType&
Bezier<DEGREE, T, POINT_DIMENSION, PointType>::controlPoints()
{
    return this->_controlPoints;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
double Bezier<DEGREE, T, POINT_DIMENSION, PointType>::operator()(size_t axis, double t)
{
    if (axis >= POINT_DIMENSION) {
        throw std::out_of_range("axis out of range");
    }

    std::vector<double> polynominalCoeffs = maths::polynomialCoeffs(DEGREE, t);

    double sum = 0;
    for (size_t i = 0; i < DEGREE + 1; ++i) {
        sum += _controlPoints[i][axis] * BINOMIAL_COEFFS[i] * polynominalCoeffs[i];
    }

    return sum;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class PointType>
PointType Bezier<DEGREE, T, POINT_DIMENSION, PointType>::operator()(double t)
{
    if (this->controlPoints().empty()) {
        throw std::out_of_range("No control points");
    }

    PointType p;

    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        p[i] = static_cast<T>(this->operator()(i, t));
    }

    return p;
}

}  // namespace robotics
#endif /* BEZIER_HPP_ */
