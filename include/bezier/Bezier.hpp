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
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

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
 *  number of control points must be equal to the degree plus 1
 *  t is the parameter of the curve, the range of t is often in the interval of [0,1], so that a new point approximated
 *  from 2 points A, B will always stay somewhat between A and B.
 *  More details should be found here: http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node2.html
 *  In this implementation, for convienience (or laziness), Eigen Matrix is used as the container of control points.
 */
template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container = Eigen::Matrix<T, POINT_DIMENSION, 1>>
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

    /**
     *  @brief function to calculate the curve Matrix to transform between a Bezier Bases and a Power Basis
     *  Though Bezier curve can be calculated as r(t) = Sigma(i = 0,n) of b_i * B_(i,n)(t)
     *  Also; r(t) = Sigma(i = 0,n) of a_i * t ^ n
     *  Supposed the b_vector T = (b_0, b_1, ..., b_n) T
     *  And the      a_vector T = (a_0, a_1, ..., a_n) T
     *  The matrix representation A(mxm) is defined as b_vector = A * a_vector (here m = DEGREE+1)
     *  We have, A_ij = jCi / jCn for i >= j; 0 otherwise
     *
     *  @return std::array<double, (DEGREE + 1) * (DEGREE + 1)>
     */
    std::array<double, (DEGREE + 1) * (DEGREE + 1)> curveMatrix() const;

    /**
     *  @brief function to calculate the inverse matrix of curve Matrix A
     *
     *  @return std::array<double, (DEGREE + 1) * (DEGREE + 1)>
     */
    std::array<double, (DEGREE + 1) * (DEGREE + 1)> inverseCurveMatrix() const;

    /**
     *  @brief overloading operator<< to quickly print out the power basis form of bezier curve
     *
     */
    friend std::ostream& operator<<(std::ostream& os, const Bezier& bezier)
    {
        assert(bezier.controlPoints().size() == DEGREE + 1);

        const size_t WIDTH = DEGREE + 1, HEIGHT = DEGREE + 1;

        const std::array<double, (DEGREE + 1) * (DEGREE + 1)> curMatrix = bezier.inverseCurveMatrix();

        VecPointType powerBasisCoeffs;
        powerBasisCoeffs.reserve(DEGREE + 1);

        for (size_t i = 0; i < HEIGHT; ++i) {
            PointType curPoints = static_cast<PointType>(PointType::Zero());
            for (size_t j = 0; j < WIDTH; ++j) {
                curPoints += curMatrix[i * WIDTH + j] * bezier.controlPoints()[j];
            }
            powerBasisCoeffs.emplace_back(curPoints);
        }

        std::vector<std::string> functionStr(POINT_DIMENSION, "");
        for (size_t i = 0; i < DEGREE + 1; ++i) {
            for (size_t j = 0; j < POINT_DIMENSION; ++j) {
                std::stringstream ss;
#if ENABLE_DEBUG
                ss << " + " << std::setw(8) << powerBasisCoeffs[i][j] << " * std::pow(t," << i << ")";
#else
                ss << " + " << std::setw(8) << powerBasisCoeffs[i][j] << " * t^" << i;
#endif  // ENABLE_DEBUG
                functionStr[j] += ss.str();
            }
        }

        os << "curve function: \n";
        for (size_t i = 0; i < POINT_DIMENSION; ++i) {
            os << "dimension number " << i << ": " << functionStr[i] << "\n";
        }

        return os;
    }

 private:
    VecPointType initializePointV();
    PointType cross(const PointType& v1, const PointType& v2) const;

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
typename Bezier<DEGREE, T, POINT_DIMENSION, Container>::PointType
Bezier<DEGREE, T, POINT_DIMENSION, Container>::cross(const PointType& v1, const PointType& v2) const
{
    if (POINT_DIMENSION != 3) {
        throw std::out_of_range("this method is for point of 3 dimensions");
    }

    PointType result;
    result << (v1[1] * v2[2] - v1[2] * v2[1]), (v1[2] * v2[0] - v1[0] * v2[2]), (v1[0] * v2[1] - v1[1] * v2[0]);

    return result;
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

        // cast into PointType type in case PointType inherits from Eigen type
        PointType b = static_cast<PointType>((tag + secondDeriv(t)).normalized());

        PointType r = static_cast<PointType>(this->cross(b, tag).normalized());

        nor = this->cross(r, tag).normalized();

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
        curvature = (this->cross(a, b).norm() / pow(a.norm(), 3));
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
        if (!maths::combinedToleranceEquals(val[i], correctVal[i])) {
            return false;
        }
    }

    return true;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
std::array<double, (DEGREE + 1) * (DEGREE + 1)> Bezier<DEGREE, T, POINT_DIMENSION, Container>::curveMatrix() const
{
    std::array<double, (DEGREE + 1) * (DEGREE + 1)> result;
    const size_t WIDTH = DEGREE + 1, HEIGHT = DEGREE + 1;

    for (size_t i = 0; i < HEIGHT; ++i) {
        for (size_t j = 0; j < WIDTH; ++j) {
            if (i >= j) {
                result[i * WIDTH + j] = maths::binomialCoeff(i, j) / maths::binomialCoeff(DEGREE, j);
            } else {
                result[i * WIDTH + j] = 0;
            }
        }
    }

    return result;
}

template <size_t DEGREE, typename T, size_t POINT_DIMENSION, class Container>
std::array<double, (DEGREE + 1) * (DEGREE + 1)>
Bezier<DEGREE, T, POINT_DIMENSION, Container>::inverseCurveMatrix() const
{
    std::array<double, (DEGREE + 1) * (DEGREE + 1)> result;
    const size_t WIDTH = DEGREE + 1, HEIGHT = DEGREE + 1;

    for (size_t i = 0; i < HEIGHT; ++i) {
        for (size_t j = 0; j < WIDTH; ++j) {
            if (i >= j) {
                result[i * WIDTH + j] =
                    std::pow(-1, i - j) * maths::binomialCoeff(i, j) * maths::binomialCoeff(DEGREE, i);
            } else {
                result[i * WIDTH + j] = 0;
            }
        }
    }

    return result;
}

}  // namespace robotics
#endif /* BEZIER_HPP_ */
