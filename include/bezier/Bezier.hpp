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

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include "bezier/NumericalMaths.hpp"
#include "bezier/PowerBasisPolynomial1D.hpp"
#include "bezier/SturmSequence.hpp"

#ifdef WITH_DEBUG
#define ENABLE_DEBUG 1
#endif

namespace robotics
{
/**
 *  @brief Implementation of Bezier Curve
 *
 *  Bezier curves are formed using Bernstern polynomial in the form of r(t) = Sigma(i = 0,n) of b_i * B_(i,n)(t)
 *  b_i is control points of arbitary dimensions; in this implementation, we focus more on 2d and 3d.
 *  B_(i,n)(t) is Bernstein polynomial.
 *  n is the degree of Bezier curve.
 *  More details should be found here: http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node2.html
 */
template <typename T, size_t POINT_DIMENSION, class Container = Eigen::Matrix<T, POINT_DIMENSION, 1>> class Bezier
{
 public:
    static constexpr T TOLERANCE = 10e-7;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! type of control points
    using PointType = Container;

    //! type of normal vector
    using Normal = PointType;

    //! type of tangent vector
    using Tangent = PointType;

    //! vector of control points
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    //! shared pointer of Bezier type
    using Ptr = std::shared_ptr<Bezier>;

    using BoundingBox = std::array<PointType, 2>;

    /**
     *  @brief Bezier Constructor
     *
     *  member variable of control points will be initialized with vector of default control points (of zeros)
     *
     *  @param degree of the Bezier curve
     */
    explicit Bezier(const size_t degree, const T tolerance = TOLERANCE);

    /**
     *  @brief Bezier Constructor
     *
     *  @param degree of the Bezier curve
     *  @param controlPoints control points to initialize member variable
     */
    Bezier(const size_t degree, const VecPointType& controlPoints, const T tolerance = TOLERANCE);

    explicit Bezier(const VecPointType& controlPoints, const T tolerance = TOLERANCE);

    /**
     *  @brief constant getter of control point member variable
     *
     *  @return const VecPointType&
     */
    const VecPointType& controlPoints() const;

    /**
     *  @brief getter for binomial coefficients
     *
     *  @return consts std::vector<double>&
     */
    const std::vector<double>& binomialCoeffs() const;

    /**
     *  @brief Bezier curve (trajectory) approximated from control points
     *
     *  @param numPoints number of points on the trajectory. notice that the first and the last points of this
     *  trajectory are the first and the last control points.
     *  @param start the lower bound range of t
     *  @param end the upper bound range of t
     *  @return VecPointType
     */
    VecPointType trajectory(const size_t numPoints, const double start = 0, const double end = 1);

    /**
     *  @brief the derivative of current Bezier class
     *
     *  @return Bezier
     */
    Bezier<T, POINT_DIMENSION, Container> derivative() const;

    /**
     *  @brief the tangent vector at t
     *
     *  @param t t value
     *  @param normalize boolean value to check whether to normalize the result or not
     *  @return Tangent
     */
    Tangent tangent(const double t, const bool normalize = true) const;

    /**
     *  @brief the normal vector at t
     *
     *  @param t t value
     *  @param normalize boolean value to check whether to normalize the result or not
     *  @return Normal
     */
    Normal normal(const double t, const bool normalize = true) const;

    /**
     *  @brief the curvature at t
     *
     *  @param t t value
     *  @return double
     */
    double curvature(const double t) const;

    /**
     *  @brief functor to calculate the value of approximated point on Bezier curve at t on the targeted axis
     *
     *  @param axis axis of the point to calculate
     *  @param t t value
     *  @return double
     */
    double operator()(const size_t axis, const double t) const;

    /**
     *  @brief functor to calculate the approximated point on Bezier curve at t
     *
     *  @param t t value
     *  @return PointType
     */
    PointType operator()(const double t) const;

    /**
     *  @brief static function to check if two PointTypes are nearly equal
     *
     *  @param val first PointType
     *  @param correctVal second PointTypes
     *  @return bool
     */
    static bool fuzzyEquals(const PointType& val, const PointType& correctVal, const T epsilon = TOLERANCE);

    /**
     *  @brief function to calculate the curve Matrix to transform between a Bezier Bases and a Power Basis
     *  Though Bezier curve can be calculated as r(t) = Sigma(i = 0,n) of b_i * B_(i,n)(t)
     *  Also; r(t) = Sigma(i = 0,n) of a_i * t ^ n
     *  Supposed the b_vector T = (b_0, b_1, ..., b_n) T
     *  And the      a_vector T = (a_0, a_1, ..., a_n) T
     *  The matrix representation A(mxm) is defined as b_vector = A * a_vector (here m = DEGREE+1)
     *  We have, A_ij = jCi / jCn for i >= j; 0 otherwise
     *
     *  @return std::vector<double>
     */
    std::vector<double> curveMatrix() const;

    /**
     *  @brief function to calculate the inverse matrix of curve Matrix A
     *
     *  @return std::vector<double>
     */
    std::vector<double> inverseCurveMatrix() const;

    /**
     *  @brief function to extract point data value on each axis
     *  Supposed a PointType p is a R(n) point. Data value of p on axis i (i = (0,n-1)) is value p[i]
     *  @param vecPointType vector of points to extract values
     *
     *  @return array of axis-wise point data value vector
     */
    std::array<std::vector<T>, POINT_DIMENSION> extractDataEachAxis(const VecPointType& vecPointType) const;

    std::vector<maths::PowerBasisPolynomial1D<T>> powerBasisForm() const;

    double closestPointToCurve(const PointType& outliner, const double start = 0, const double end = 1) const;

    BoundingBox estimateBoundingBox(double start = 0, double end = 1) const;

    double approximateLength(const VecPointType& trajectory) const;

    /**
     *  @brief overloading operator<< to quickly print out the power basis form of bezier curve
     *
     */
    friend std::ostream& operator<<(std::ostream& os, const Bezier& bezier)
    {
        std::vector<maths::PowerBasisPolynomial1D<T>> powerBases = bezier.powerBasisForm();

        os << "curve function: \n";
        for (size_t i = 0; i < POINT_DIMENSION; ++i) {
            os << "dimension number " << i << ": " << powerBases[i] << "\n";
        }

        return os;
    }

 private:
    /**
     *  @brief manually-defined cross product function as Eigen's cross product has constraint on dimension.
     *
     *  @param v1 first point
     *  @param v2 second point
     *  @return PointType
     */
    PointType cross(const PointType& v1, const PointType& v2) const;

 private:
    //! control points that define characteristic of Bezier curves
    VecPointType _controlPoints;
    //! degree of the Bezier curve
    const size_t _degree;
    //! vector to hold all the (n+1) binomial coefficients of kCn
    const std::vector<double> _binomialCoeffs;

    T _tolerance;
};

template <typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<T, POINT_DIMENSION, Container>::PointType
Bezier<T, POINT_DIMENSION, Container>::cross(const PointType& v1, const PointType& v2) const
{
    if (POINT_DIMENSION != 3) {
        throw std::out_of_range("this method is for point of 3 dimensions");
    }

    PointType result;
    result << (v1[1] * v2[2] - v1[2] * v2[1]), (v1[2] * v2[0] - v1[0] * v2[2]), (v1[0] * v2[1] - v1[1] * v2[0]);

    return result;
}

template <typename T, size_t POINT_DIMENSION, class Container>
Bezier<T, POINT_DIMENSION, Container>::Bezier(const size_t degree, const T tolerance)
    : _controlPoints(VecPointType(degree + 1, static_cast<PointType>(PointType::Zero()))), _degree(degree),
      _tolerance(tolerance), _binomialCoeffs(maths::binomialCoeffs(degree))
{
}

template <typename T, size_t POINT_DIMENSION, class Container>
Bezier<T, POINT_DIMENSION, Container>::Bezier(const size_t degree, const VecPointType& controlPoints, const T tolerance)
    : _controlPoints(controlPoints), _degree(degree), _tolerance(tolerance),
      _binomialCoeffs(maths::binomialCoeffs(degree))
{
}

template <typename T, size_t POINT_DIMENSION, class Container>
Bezier<T, POINT_DIMENSION, Container>::Bezier(const VecPointType& controlPoints, const T tolerance)
    : _controlPoints(controlPoints), _degree(controlPoints.size() - 1), _tolerance(tolerance),
      _binomialCoeffs(maths::binomialCoeffs(controlPoints.size() - 1))
{
    assert(controlPoints.size() > 0);
}

template <typename T, size_t POINT_DIMENSION, class Container>
const typename Bezier<T, POINT_DIMENSION, Container>::VecPointType&
Bezier<T, POINT_DIMENSION, Container>::controlPoints() const
{
    return this->_controlPoints;
}

template <typename T, size_t POINT_DIMENSION, class Container>
const std::vector<double>& Bezier<T, POINT_DIMENSION, Container>::binomialCoeffs() const
{
    return this->_binomialCoeffs;
}

template <typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<T, POINT_DIMENSION, Container>::VecPointType
Bezier<T, POINT_DIMENSION, Container>::trajectory(const size_t numPoints, const double start, const double end)
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

template <typename T, size_t POINT_DIMENSION, class Container>
Bezier<T, POINT_DIMENSION, Container> Bezier<T, POINT_DIMENSION, Container>::derivative() const
{
    if (this->_degree == 0) {
        throw std::out_of_range("degree must be more than 0");
    }

    VecPointType deriBControlPoints;
    deriBControlPoints.reserve(this->_degree);
    for (size_t i = 0; i < this->_degree; ++i) {
        deriBControlPoints.emplace_back(this->_degree * (this->_controlPoints[i + 1] - this->_controlPoints[i]));
    }

    return Bezier<T, POINT_DIMENSION, Container>(this->_degree - 1, deriBControlPoints);
}

template <typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<T, POINT_DIMENSION, Container>::Tangent
Bezier<T, POINT_DIMENSION, Container>::tangent(const double t, const bool normalize) const
{
    Tangent tag;
    Bezier<T, POINT_DIMENSION, Container> derivativeBezier = this->derivative();
    tag = derivativeBezier(t);
    if (normalize) {
        tag.normalize();
    }

    return tag;
}

template <typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<T, POINT_DIMENSION, Container>::Normal
Bezier<T, POINT_DIMENSION, Container>::normal(const double t, const bool normalize) const
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
        // Here is the naive implementation for 3d normal
        // Algorithm approach can be Rotation Minimising Frames
        // https://pomax.github.io/bezierinfo/#pointvectors

        Bezier<T, POINT_DIMENSION, Container> secondDerivBezier = this->derivative().derivative();

        PointType b = static_cast<PointType>((tag + secondDerivBezier(t)).normalized());

        PointType r = static_cast<PointType>(this->cross(b, tag).normalized());

        nor = this->cross(r, tag);

        if (normalize) {
            nor.normalize();
        }
    }

    return nor;
}

template <typename T, size_t POINT_DIMENSION, class Container>
double Bezier<T, POINT_DIMENSION, Container>::curvature(const double t) const
{
    if (this->_degree < 2 || (POINT_DIMENSION != 2 && POINT_DIMENSION != 3)) {
        std::string msg = "This method is for degree more than 1 and";
        msg += " control points for dimension of 2 or 3";
        throw std::out_of_range(msg);
    }

    Bezier<T, POINT_DIMENSION, Container> derivativeBezier = this->derivative();
    Bezier<T, POINT_DIMENSION, Container> secondDerivBezier = derivativeBezier.derivative();

    PointType a = derivativeBezier(t);
    PointType b = secondDerivBezier(t);
    double curvature;

    if (POINT_DIMENSION == 2) {
        curvature = (a[0] * b[1] - a[1] * b[0]) / pow(a.norm(), 3);
    }

    if (POINT_DIMENSION == 3) {
        curvature = (this->cross(a, b).norm() / pow(a.norm(), 3));
    }

    return curvature;
}

template <typename T, size_t POINT_DIMENSION, class Container>
double Bezier<T, POINT_DIMENSION, Container>::operator()(const size_t axis, const double t) const
{
    if (axis >= POINT_DIMENSION) {
        throw std::out_of_range("axis out of range");
    }

    if (this->_controlPoints.size() < this->_degree + 1) {
        throw std::out_of_range("number of control points must be more than degree");
    }

    std::vector<double> polynominalCoeffs = maths::polynomialCoeffs(this->_degree, t);

    double sum = 0;
    for (size_t i = 0; i < this->_degree + 1; ++i) {
        sum += this->_controlPoints[i][axis] * _binomialCoeffs[i] * polynominalCoeffs[i];
    }

    return sum;
}

template <typename T, size_t POINT_DIMENSION, class Container>
Container Bezier<T, POINT_DIMENSION, Container>::operator()(const double t) const
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

template <typename T, size_t POINT_DIMENSION, class Container>
bool Bezier<T, POINT_DIMENSION, Container>::fuzzyEquals(const PointType& val, const PointType& correctVal,
                                                        const T epsilon)
{
    for (size_t i = 0; i < val.size(); ++i) {
        if (!maths::combinedToleranceEquals(val[i], correctVal[i], epsilon)) {
            return false;
        }
    }

    return true;
}

template <typename T, size_t POINT_DIMENSION, class Container>
std::vector<double> Bezier<T, POINT_DIMENSION, Container>::curveMatrix() const
{
    const size_t WIDTH = this->_degree + 1, HEIGHT = this->_degree + 1;
    std::vector<double> result(HEIGHT * WIDTH, 0.0);

    for (size_t i = 0; i < HEIGHT; ++i) {
        for (size_t j = 0; j < WIDTH; ++j) {
            if (i >= j) {
                result[i * WIDTH + j] = maths::binomialCoeff(i, j) / maths::binomialCoeff(this->_degree, j);
            }
        }
    }

    return result;
}

template <typename T, size_t POINT_DIMENSION, class Container>
std::vector<double> Bezier<T, POINT_DIMENSION, Container>::inverseCurveMatrix() const
{
    const size_t WIDTH = this->_degree + 1, HEIGHT = this->_degree + 1;
    std::vector<double> result(HEIGHT * WIDTH, 0.0);

    for (size_t i = 0; i < HEIGHT; ++i) {
        for (size_t j = 0; j < WIDTH; ++j) {
            if (i >= j) {
                result[i * WIDTH + j] =
                    std::pow(-1, i - j) * maths::binomialCoeff(i, j) * maths::binomialCoeff(this->_degree, i);
            }
        }
    }

    return result;
}

template <typename T, size_t POINT_DIMENSION, class Container>
std::array<std::vector<T>, POINT_DIMENSION>
Bezier<T, POINT_DIMENSION, Container>::extractDataEachAxis(const VecPointType& vecPointType) const
{
    std::array<std::vector<T>, POINT_DIMENSION> result;

    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        std::vector<T> curV;
        curV.reserve(vecPointType.size());
        std::transform(vecPointType.begin(), vecPointType.end(), std::back_inserter(curV),
                       [&i](const PointType& p) { return p[i]; });
        result[i] = curV;
    }

    return result;
}

template <typename T, size_t POINT_DIMENSION, class Container>
std::vector<maths::PowerBasisPolynomial1D<T>> Bezier<T, POINT_DIMENSION, Container>::powerBasisForm() const
{
    assert(this->controlPoints().size() == this->_degree + 1);

    const size_t WIDTH = this->_degree + 1, HEIGHT = this->_degree + 1;

    const std::vector<double> curMatrix = this->inverseCurveMatrix();

    VecPointType powerBasisCoeffsVec;
    powerBasisCoeffsVec.reserve(this->_degree + 1);

    for (size_t i = 0; i < HEIGHT; ++i) {
        PointType curPoints = static_cast<PointType>(PointType::Zero());
        for (size_t j = 0; j < WIDTH; ++j) {
            curPoints += curMatrix[i * WIDTH + j] * this->controlPoints()[j];
        }
        powerBasisCoeffsVec.emplace_back(curPoints);
    }

    auto powerBasisCoeffs = this->extractDataEachAxis(powerBasisCoeffsVec);
    std::vector<maths::PowerBasisPolynomial1D<T>> result;
    result.reserve(POINT_DIMENSION);

    for (const auto& powerBasisCoeff : powerBasisCoeffs) {
        result.emplace_back(maths::PowerBasisPolynomial1D<T>(powerBasisCoeff));
    }

    return result;
}

template <typename T, size_t POINT_DIMENSION, class Container>
double Bezier<T, POINT_DIMENSION, Container>::closestPointToCurve(const PointType& outliner, const double start,
                                                                  const double end) const
{
    // solve P(t) = (Q(t) - outliner).dot(Q'(t)') = 0
    // Q1(t) = (Q(t) - outliner)
    // Q2(t) = Q'(t)

    this->_controlPoints;
    VecPointType q1ControlPoints;
    q1ControlPoints.reserve(this->_controlPoints.size());
    std::transform(this->_controlPoints.begin(), this->_controlPoints.end(), std::back_inserter(q1ControlPoints),
                   [&outliner](const PointType& p) { return p - outliner; });

    const Bezier Q1(this->_degree, q1ControlPoints);
    const Bezier Q2 = this->derivative();

    const auto q1BasisPowerForm = Q1.powerBasisForm();
    const auto q2BasisPowerForm = Q2.powerBasisForm();
    maths::PowerBasisPolynomial1D<T> poly({0}, false, this->_tolerance);

    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        poly += q1BasisPowerForm[i].multiply(q2BasisPowerForm[i]);
    }

    const maths::SturmSequence<T> ss(poly, this->_tolerance);

    const auto roots = ss.solveLocalMinium(this->_tolerance, 1.f - this->_tolerance);

    std::vector<double> shortestVals;
    std::vector<double> shortestDistances;
    shortestVals.reserve(roots.size() + 2);
    shortestDistances.reserve(roots.size() + 2);

    shortestVals.emplace_back(start);
    shortestDistances.emplace_back((this->operator()(start) - outliner).norm());

    for (const auto& root : roots) {
        shortestVals.emplace_back(root);
        shortestDistances.emplace_back((this->operator()(root) - outliner).norm());
    }

    shortestVals.emplace_back(end);
    shortestDistances.emplace_back((this->operator()(end) - outliner).norm());

    int shortestValIndx =
        std::distance(shortestDistances.begin(), std::min_element(shortestDistances.begin(), shortestDistances.end()));

    return shortestVals[shortestValIndx];
}

template <typename T, size_t POINT_DIMENSION, class Container>
typename Bezier<T, POINT_DIMENSION, Container>::BoundingBox
Bezier<T, POINT_DIMENSION, Container>::estimateBoundingBox(double start, double end) const
{
    auto derivPowerBases = this->derivative().powerBasisForm();
    PointType topLeft, bottomRight;

    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        const maths::PowerBasisPolynomial1D<T> axisPoly = derivPowerBases[i];
        const maths::SturmSequence<T> ss(axisPoly, this->_tolerance);
        const auto roots = ss.solve(start + this->_tolerance, end - this->_tolerance);
        std::vector<double> axisVals;
        axisVals.reserve(roots.size() + 2);

        axisVals.emplace_back(this->operator()(i, start));

        std::transform(roots.begin(), roots.end(), std::back_inserter(axisVals),
                       [&i, this](const T val) { return this->operator()(i, val); });

        axisVals.emplace_back(this->operator()(i, end));

        const auto minMax = std::minmax_element(axisVals.begin(), axisVals.end());
        topLeft[i] = *minMax.first;
        bottomRight[i] = *minMax.second;
    }

    return {topLeft, bottomRight};
}

template <typename T, size_t POINT_DIMENSION, class Container>
double Bezier<T, POINT_DIMENSION, Container>::approximateLength(const VecPointType& trajectory) const
{
    double length = 0;
    for (auto it = trajectory.cbegin(); it != trajectory.cend() - 1; ++it) {
        length += (*it - *std::next(it)).norm();
    }

    return length;
}

}  // namespace robotics
