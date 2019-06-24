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
#include <Eigen/Eigen>
#include <algorithm>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace robotics
{
template <typename T, size_t POINT_DIMENSION> using DefaultVector = Eigen::Matrix<T, POINT_DIMENSION, 1>;

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container = DefaultVector<T, POINT_DIMENSION>>
class Bezier
{
 public:
    static const std::vector<double> BINOMIAL_COEFFS;
    static constexpr int degree = DEGREE;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = Container;
    using Normal = PointType;
    using Tangent = PointType;
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using Ptr = std::shared_ptr<Bezier>;

    Bezier();
    explicit Bezier(const VecPointType& controlPoints);
    const VecPointType& controlPoints() const;
    VecPointType& controlPoints();
    VecPointType trajectory(size_t numPoints, double start = 0, double end = 1);

    Bezier<DEGREE - 1, T, POINT_DIMENSION, Container> derivative() const;
    Tangent tangent(double t, bool normalize = true) const;
    Normal normal(double t, bool normalize = true) const;
    double curvature(double t) const;

    double operator()(size_t axis, double t);
    PointType operator()(double t);
    static bool fuzzyEquals(PointType val, PointType correctVal);

 private:
    VecPointType initializePointV();

 private:
    VecPointType _controlPoints;
};

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
const std::vector<double>
    Bezier<DEGREE, T, POINT_DIMENSION, Container>::BINOMIAL_COEFFS = maths::binomialCoeffs(DEGREE);

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<DEGREE, T, POINT_DIMENSION, Container>::VecPointType
Bezier<DEGREE, T, POINT_DIMENSION, Container>::initializePointV()
{
    std::vector<double> v(DEGREE + 1, 0);
    PointType defaultP(v.data());
    VecPointType pointV(DEGREE + 1, defaultP);

    return pointV;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
Bezier<DEGREE, T, POINT_DIMENSION, Container>::Bezier() : _controlPoints(initializePointV())
{
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
Bezier<DEGREE, T, POINT_DIMENSION, Container>::Bezier(const VecPointType& controlPoints) : _controlPoints(controlPoints)
{
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
const typename Bezier<DEGREE, T, POINT_DIMENSION, Container>::VecPointType&
Bezier<DEGREE, T, POINT_DIMENSION, Container>::controlPoints() const
{
    return this->_controlPoints;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<DEGREE, T, POINT_DIMENSION, Container>::VecPointType&
Bezier<DEGREE, T, POINT_DIMENSION, Container>::controlPoints()
{
    return this->_controlPoints;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<DEGREE, T, POINT_DIMENSION, Container>::VecPointType
Bezier<DEGREE, T, POINT_DIMENSION, Container>::trajectory(size_t numPoints, double start, double end)
{
    if (numPoints == 0) {
        throw std::out_of_range("Number of points must be more than 0");
    }

    VecPointType v;
    v.reserve(numPoints + 1);

    double delta = (end - start) / numPoints;
    for (size_t i = 0; i <= numPoints; ++i) {
        double t = start + i * delta;
        v.emplace_back(this->operator()(t));
    }

    return v;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
Bezier<DEGREE - 1, T, POINT_DIMENSION, Container> Bezier<DEGREE, T, POINT_DIMENSION, Container>::derivative() const
{
    if (DEGREE == 0) {
        throw std::out_of_range("degree must be more than 0");
    }

    Bezier<DEGREE - 1, T, POINT_DIMENSION, Container> deriB;

    for (size_t i = 0; i < DEGREE; ++i) {
        deriB.controlPoints()[i] = DEGREE * (this->_controlPoints[i + 1] - this->_controlPoints[i]);
    }

    return deriB;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<DEGREE, T, POINT_DIMENSION, Container>::Tangent
Bezier<DEGREE, T, POINT_DIMENSION, Container>::tangent(double t, bool normalize) const
{
    Tangent tag;
    Bezier<DEGREE - 1, T, POINT_DIMENSION, Container> derivative = this->derivative();
    tag = derivative(t);
    if (normalize) {
        tag.normalize();
    }

    return tag;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<DEGREE, T, POINT_DIMENSION, Container>::Normal
Bezier<DEGREE, T, POINT_DIMENSION, Container>::normal(double t, bool normalize) const
{
    if (POINT_DIMENSION != 2 && POINT_DIMENSION != 3) {
        throw std::out_of_range("This method is for control points of dimension of 2 or 3");
    }

    Tangent tag = this->tangent(t, normalize);

    Normal nor;

    if (POINT_DIMENSION == 2) {
        nor << -(tag[1]), tag[0];
    }

    if (POINT_DIMENSION == 3) {
        assert(POINT_DIMENSION == 3);
        // Here is the naive implementation for 3d normal
        // Algorithm approach can be Rotation Minimising Frames
        // https://pomax.github.io/bezierinfo/#pointvectors

        Bezier<DEGREE - 2, T, POINT_DIMENSION, Container> secondDeriv = this->derivative().derivative();

        PointType b = (tag + secondDeriv(t)).normalized();

        PointType r = b.cross(tag).normalized();

        nor = r.cross(tag);

        if (normalize) {
            nor.normalize();
        }
    }

    return nor;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
double Bezier<DEGREE, T, POINT_DIMENSION, Container>::curvature(double t) const
{
    if (DEGREE < 2 || (POINT_DIMENSION != 2 && POINT_DIMENSION != 3)) {
        std::string msg = "This method is for degree more than 1 and";
        msg += " control points for dimension of 2 or 3";
        throw std::out_of_range(msg);
    }

    Bezier<DEGREE - 1, T, POINT_DIMENSION, Container> derivative = this->derivative();
    Bezier<DEGREE - 2, T, POINT_DIMENSION, Container> secondDeriv = derivative.derivative();

    PointType a = derivative(t);
    PointType b = secondDeriv(t);
    double curvature;

    if (POINT_DIMENSION == 2) {
        curvature = (a[0] * b[1] - a[1] * b[0]) / pow(a.norm(), 3);
    }

    if (POINT_DIMENSION == 3) {
        curvature = (a.cross(b)).norm() / pow(a.norm(), 3);
    }

    return curvature;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
double Bezier<DEGREE, T, POINT_DIMENSION, Container>::operator()(size_t axis, double t)
{
    if (axis >= POINT_DIMENSION) {
        throw std::out_of_range("axis out of range");
    }

    if (this->_controlPoints.size() < DEGREE + 1) {
        throw std::out_of_range("number of control points must be more than degree");
    }

    std::vector<double> polynominalCoeffs = maths::polynomialCoeffs(DEGREE, t);

    double sum = 0;
    for (size_t i = 0; i < DEGREE + 1; ++i) {
        sum += this->_controlPoints[i][axis] * BINOMIAL_COEFFS[i] * polynominalCoeffs[i];
    }

    return sum;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
Container Bezier<DEGREE, T, POINT_DIMENSION, Container>::operator()(double t)
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

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
bool Bezier<DEGREE, T, POINT_DIMENSION, Container>::fuzzyEquals(PointType val, PointType correctVal)
{
    for (size_t i = 0; i < val.size(); ++i) {
        if (!maths::fuzzyEquals(val[i], correctVal[i])) {
            return false;
        }
    }

    return true;
}

}  // namespace robotics
#endif /* BEZIER_HPP_ */
