/**
 * @file    PowerBasisPolyNomial1D.hpp
 *
 * @author  btran
 *
 * @date    2019-08-11
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include "bezier/NumericalMaths.hpp"

namespace robotics
{
namespace maths
{
template <typename DATA_TYPE = double> class PowerBasisPolynomial1D
{
 public:
    static constexpr DATA_TYPE TOLERANCE = 10e-7;

    using QuotientRemainder = std::pair<PowerBasisPolynomial1D, PowerBasisPolynomial1D>;

    explicit PowerBasisPolynomial1D(const std::vector<DATA_TYPE>& coeffs, const bool normalized = false,
                                    const DATA_TYPE tolerance = TOLERANCE);

    PowerBasisPolynomial1D(const PowerBasisPolynomial1D& other, const bool normalized = false);

    DATA_TYPE operator()(const DATA_TYPE t) const;

    const std::vector<DATA_TYPE>& coeffs() const;

    const size_t degree() const;

    bool isZero() const;

    PowerBasisPolynomial1D add(const PowerBasisPolynomial1D& other) const;

    PowerBasisPolynomial1D& operator+=(const PowerBasisPolynomial1D& other);

    PowerBasisPolynomial1D multiply(const PowerBasisPolynomial1D& other) const;

    PowerBasisPolynomial1D multiply(const DATA_TYPE scalar) const;

    PowerBasisPolynomial1D derivative() const;

    QuotientRemainder divide(const PowerBasisPolynomial1D& other) const;

    PowerBasisPolynomial1D operator%(const PowerBasisPolynomial1D& other) const;

    static PowerBasisPolynomial1D gcd(const PowerBasisPolynomial1D& firstPoly,
                                      const PowerBasisPolynomial1D& secondPoly);

    bool isMonic() const;

    PowerBasisPolynomial1D monicized() const;

    friend std::ostream& operator<<(std::ostream& os, const PowerBasisPolynomial1D& polynomial)
    {
        assert(polynomial.degree() >= 0);

        os << "Polynomial function: \n";
        for (size_t i = 0; i < polynomial.coeffs().size(); ++i) {
            if (combinedToleranceEquals(polynomial.coeffs()[i], static_cast<DATA_TYPE>(0), polynomial._tolerance)) {
                continue;
            }

#if ENABLE_DEBUG
            os << " + " << std::setw(8) << polynomial.coeffs()[i] << " * std::pow(t," << i << ")";
#else
            os << " + " << std::setw(8) << polynomial.coeffs()[i] << " * t^" << i;
#endif  // ENABLE_DEBUG
        }

        return os;
    }

 private:
    void removeLeadingZeros();
    void monicize();

 private:
    std::vector<DATA_TYPE> _coeffs;
    DATA_TYPE _tolerance;
};

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>::PowerBasisPolynomial1D(const std::vector<DATA_TYPE>& coeffs, const bool normalized,
                                                          const DATA_TYPE tolerance)
    : _coeffs(coeffs), _tolerance(tolerance)
{
    assert(this->_coeffs.size() > 0);
    this->removeLeadingZeros();

    if (normalized && !this->isMonic()) {
        this->monicize();
    }
}

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>::PowerBasisPolynomial1D(const PowerBasisPolynomial1D<DATA_TYPE>& other,
                                                          const bool normalized)
{
    this->_coeffs = other._coeffs;
    this->_tolerance = other._tolerance;

    if (normalized && !this->isMonic()) {
        this->monicize();
    }
}

template <typename DATA_TYPE> DATA_TYPE PowerBasisPolynomial1D<DATA_TYPE>::operator()(const DATA_TYPE t) const
{
    DATA_TYPE result = static_cast<DATA_TYPE>(0);

    for (size_t i = 0; i < this->_coeffs.size(); ++i) {
        result += this->_coeffs[i] * std::pow(t, i);
    }

    return result;
}

template <typename DATA_TYPE> const std::vector<DATA_TYPE>& PowerBasisPolynomial1D<DATA_TYPE>::coeffs() const
{
    return this->_coeffs;
}

template <typename DATA_TYPE> const size_t PowerBasisPolynomial1D<DATA_TYPE>::degree() const
{
    return this->_coeffs.size() - 1;
}

template <typename DATA_TYPE> bool PowerBasisPolynomial1D<DATA_TYPE>::isZero() const
{
    return (this->degree() == 0 &&
            combinedToleranceEquals(this->_coeffs.back(), static_cast<DATA_TYPE>(0), this->_tolerance));
}

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>::add(const PowerBasisPolynomial1D<DATA_TYPE>& other) const
{
    std::vector<DATA_TYPE> biggerDegCoeffs =
        this->_coeffs.size() >= other._coeffs.size() ? this->_coeffs : other._coeffs;
    std::vector<DATA_TYPE> smallerDegCoeffs =
        this->_coeffs.size() < other._coeffs.size() ? this->_coeffs : other._coeffs;

    for (size_t i = 0; i < smallerDegCoeffs.size(); ++i) {
        biggerDegCoeffs[i] += smallerDegCoeffs[i];
    }

    return PowerBasisPolynomial1D<DATA_TYPE>(biggerDegCoeffs);
}

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>& PowerBasisPolynomial1D<DATA_TYPE>::operator+=(const PowerBasisPolynomial1D& other)
{
    *this = this->add(other);
    return *this;
}

template <typename DATA_TYPE = double>
PowerBasisPolynomial1D<DATA_TYPE> addPolynomials(const PowerBasisPolynomial1D<DATA_TYPE>& p1,
                                                 const PowerBasisPolynomial1D<DATA_TYPE>& p2)
{
    return p1.add(p2);
}

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>::multiply(const PowerBasisPolynomial1D<DATA_TYPE>& other) const
{
    // naive approach for now
    // better approach using fft
    // [source](http://people.csail.mit.edu/madhu/ST12/scribe/lect06.pdf)
    std::vector<DATA_TYPE> resultCoeffs(this->_coeffs.size() + other._coeffs.size(), 0.0);

    for (size_t i = 0; i < this->_coeffs.size(); ++i) {
        for (size_t io = 0; io < other._coeffs.size(); ++io) {
            resultCoeffs[i + io] += this->_coeffs[i] * other._coeffs[io];
        }
    }

    return PowerBasisPolynomial1D<DATA_TYPE>(resultCoeffs);
}

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE> PowerBasisPolynomial1D<DATA_TYPE>::multiply(const DATA_TYPE scalar) const
{
    std::vector<DATA_TYPE> resultCoeffs;
    resultCoeffs.reserve(this->_coeffs.size());

    std::transform(this->_coeffs.begin(), this->_coeffs.end(), std::back_inserter(resultCoeffs),
                   [&scalar](const DATA_TYPE val) { return val * scalar; });

    return PowerBasisPolynomial1D<DATA_TYPE>(resultCoeffs);
}

template <typename DATA_TYPE = double>
PowerBasisPolynomial1D<DATA_TYPE> multiplyPolynomials(const PowerBasisPolynomial1D<DATA_TYPE>& p1,
                                                      const PowerBasisPolynomial1D<DATA_TYPE>& p2)
{
    return p1.multiply(p2);
}

template <typename DATA_TYPE> PowerBasisPolynomial1D<DATA_TYPE> PowerBasisPolynomial1D<DATA_TYPE>::derivative() const
{
    assert(this->_coeffs.size() > 0);
    if (this->_coeffs.size() == 1) {
        return PowerBasisPolynomial1D(std::vector<DATA_TYPE>{0});
    }

    std::vector<DATA_TYPE> derivCoeffs(this->_coeffs.begin() + 1, this->_coeffs.end());
    for (size_t i = 0; i < derivCoeffs.size(); ++i) {
        derivCoeffs[i] *= (i + 1);
    }

    return PowerBasisPolynomial1D(derivCoeffs);
}

template <typename DATA_TYPE>
typename PowerBasisPolynomial1D<DATA_TYPE>::QuotientRemainder
PowerBasisPolynomial1D<DATA_TYPE>::divide(const PowerBasisPolynomial1D& other) const
{
    // naive approach for now
    // [source](http://web.cs.iastate.edu/~cs577/handouts/polydivide.pdf)
    if (other.degree() == 0) {
        if (combinedToleranceEquals(other._coeffs.front(), static_cast<DATA_TYPE>(0), this->_tolerance)) {
            throw std::runtime_error("cannot divide by 0");
        } else {
            return std::make_pair(this->multiply(1.0 / other._coeffs.front()),
                                  PowerBasisPolynomial1D(std::vector<DATA_TYPE>{0}));
        }
    }

    if (this->degree() < other.degree()) {
        return std::make_pair(PowerBasisPolynomial1D(std::vector<DATA_TYPE>{0}), *this);
    }

    // N / D == q; N % D == r
    std::vector<DATA_TYPE> N = this->_coeffs;
    std::vector<DATA_TYPE> D = other._coeffs;
    std::vector<DATA_TYPE> q(this->degree() - other.degree() + 1, 0);
    std::vector<DATA_TYPE> r;
    r.reserve(other.degree());

    for (int k = (this->degree() - other.degree()); k >= 0; --k) {
        q[k] = N[other.degree() + k] / D.back();
        if (combinedToleranceEquals(q[k], static_cast<DATA_TYPE>(0), this->_tolerance)) {
            q[k] = static_cast<DATA_TYPE>(0);
        }

        for (int j = other.degree() + k - 1; j >= k; --j) {
            N[j] -= q[k] * D[j - k];
        }
    }

    std::copy(N.begin(), N.begin() + other.degree(), std::back_inserter(r));

    return std::make_pair(PowerBasisPolynomial1D(q), PowerBasisPolynomial1D(r));
}

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE> PowerBasisPolynomial1D<DATA_TYPE>::
operator%(const PowerBasisPolynomial1D& other) const
{
    return this->divide(other).second;
}

template <typename DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>
PowerBasisPolynomial1D<DATA_TYPE>::gcd(const PowerBasisPolynomial1D<DATA_TYPE>& firstPoly,
                                       const PowerBasisPolynomial1D<DATA_TYPE>& secondPoly)
{
    if (firstPoly.degree() < secondPoly.degree()) {
        return gcd(secondPoly, firstPoly);
    }

    if (secondPoly.isZero()) {
        return PowerBasisPolynomial1D<DATA_TYPE>(firstPoly, true);
    }

    PowerBasisPolynomial1D<DATA_TYPE> remainder = firstPoly % secondPoly;

    return gcd(secondPoly, remainder);
}

template <typename DATA_TYPE> bool PowerBasisPolynomial1D<DATA_TYPE>::isMonic() const
{
    return combinedToleranceEquals(std::fabs(this->_coeffs.back()), static_cast<DATA_TYPE>(1), this->_tolerance);
}

template <typename DATA_TYPE> PowerBasisPolynomial1D<DATA_TYPE> PowerBasisPolynomial1D<DATA_TYPE>::monicized() const
{
    if (this->isMonic()) {
        return *this;
    }

    return PowerBasisPolynomial1D(*this, true);
}

template <typename DATA_TYPE> void PowerBasisPolynomial1D<DATA_TYPE>::monicize()
{
    if (this->isZero()) {
        return;
    }

    const DATA_TYPE biggestDegCoeffAbs = std::fabs(this->_coeffs.back());

    for (auto it = this->_coeffs.begin(); it != this->_coeffs.end(); ++it) {
        *it /= biggestDegCoeffAbs;
    }
}

template <typename DATA_TYPE> void PowerBasisPolynomial1D<DATA_TYPE>::removeLeadingZeros()
{
    while (combinedToleranceEquals(this->_coeffs.back(), static_cast<DATA_TYPE>(0), this->_tolerance) &&
           this->_coeffs.size() > 1) {
        this->_coeffs.pop_back();
    }
    if (combinedToleranceEquals(this->_coeffs.back(), static_cast<DATA_TYPE>(0), this->_tolerance)) {
        this->_coeffs.back() = static_cast<DATA_TYPE>(0);
    }
}

}  // namespace maths
}  // namespace robotics
