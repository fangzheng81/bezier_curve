/**
 * @file    PowerBasisPolynomial.hpp
 *
 * @author  btran
 *
 * @date    2019-08-11
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <bezier/Types.hpp>
#include <cassert>
#include <deque>
#include <iomanip>
#include <sstream>
#include <vector>

namespace robotics
{
namespace maths
{
template <typename DATA_TYPE = double> class PowerBasisPolynomial
{
 public:
    explicit PowerBasisPolynomial(const std::deque<DATA_TYPE>& coeffs);

    explicit PowerBasisPolynomial(const std::vector<DATA_TYPE>& coeffs);

    const std::deque<DATA_TYPE>& coeffs() const;

    const size_t degree() const;

    PowerBasisPolynomial add(const PowerBasisPolynomial& other) const;

    PowerBasisPolynomial multiply(const PowerBasisPolynomial& other) const;

    PowerBasisPolynomial derivative() const;

    friend std::ostream& operator<<(std::ostream& os, const PowerBasisPolynomial& polynomial)
    {
        assert(polynomial.degree() >= 0);

        os << "Polynomial function: \n";
        for (size_t i = 0; i < polynomial.coeffs().size(); ++i) {
            if (fuzzyEquals(polynomial.coeffs()[i], 0.0)) {
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

 private:
    std::deque<DATA_TYPE> _coeffs;
};

template <typename DATA_TYPE>
PowerBasisPolynomial<DATA_TYPE>::PowerBasisPolynomial(const std::deque<DATA_TYPE>& coeffs) : _coeffs(coeffs)
{
    assert(this->_coeffs.size() > 0);
    this->removeLeadingZeros();
}

template <typename DATA_TYPE>
PowerBasisPolynomial<DATA_TYPE>::PowerBasisPolynomial(const std::vector<DATA_TYPE>& coeffs)
    : _coeffs(std::deque<DATA_TYPE>(coeffs.begin(), coeffs.end()))
{
    assert(this->_coeffs.size() > 0);
    this->removeLeadingZeros();
}

template <typename DATA_TYPE> const std::deque<DATA_TYPE>& PowerBasisPolynomial<DATA_TYPE>::coeffs() const
{
    return this->_coeffs;
}

template <typename DATA_TYPE> const size_t PowerBasisPolynomial<DATA_TYPE>::degree() const
{
    return this->_coeffs.size() - 1;
}

template <typename DATA_TYPE>
PowerBasisPolynomial<DATA_TYPE> PowerBasisPolynomial<DATA_TYPE>::add(const PowerBasisPolynomial<DATA_TYPE>& other) const
{
    std::deque<DATA_TYPE> biggerDegCoeffs =
        this->_coeffs.size() >= other._coeffs.size() ? this->_coeffs : other._coeffs;
    std::deque<DATA_TYPE> smallerDegCoeffs =
        this->_coeffs.size() < other._coeffs.size() ? this->_coeffs : other._coeffs;

    for (size_t i = 0; i < smallerDegCoeffs.size(); ++i) {
        biggerDegCoeffs[i] += smallerDegCoeffs[i];
    }

    return PowerBasisPolynomial<DATA_TYPE>(biggerDegCoeffs);
}

template <typename DATA_TYPE = double>
PowerBasisPolynomial<DATA_TYPE> addPolynomials(const PowerBasisPolynomial<DATA_TYPE>& p1,
                                               const PowerBasisPolynomial<DATA_TYPE>& p2)
{
    return p1.add(p2);
}

template <typename DATA_TYPE>
PowerBasisPolynomial<DATA_TYPE>
PowerBasisPolynomial<DATA_TYPE>::multiply(const PowerBasisPolynomial<DATA_TYPE>& other) const
{
    std::deque<DATA_TYPE> resultCoeffs(this->_coeffs.size() + other._coeffs.size(), 0.0);

    for (size_t i = 0; i < this->_coeffs.size(); ++i) {
        for (size_t io = 0; io < other._coeffs.size(); ++io) {
            resultCoeffs[i + io] += this->_coeffs[i] * other._coeffs[io];
        }
    }

    return PowerBasisPolynomial<DATA_TYPE>(resultCoeffs);
}

template <typename DATA_TYPE = double>
PowerBasisPolynomial<DATA_TYPE> multiplyPolynomials(const PowerBasisPolynomial<DATA_TYPE>& p1,
                                                    const PowerBasisPolynomial<DATA_TYPE>& p2)
{
    return p1.multiply(p2);
}

template <typename DATA_TYPE> PowerBasisPolynomial<DATA_TYPE> PowerBasisPolynomial<DATA_TYPE>::derivative() const
{
    assert(this->_coeffs.size() > 0);
    if (this->_coeffs.size() == 1) {
        return PowerBasisPolynomial(std::deque<DATA_TYPE>{0});
    }

    std::deque<DATA_TYPE> derivCoeffs(this->_coeffs.begin() + 1, this->_coeffs.end());
    for (size_t i = 0; i < derivCoeffs.size(); ++i) {
        derivCoeffs[i] *= (i + 1);
    }

    return PowerBasisPolynomial(derivCoeffs);
}

template <typename DATA_TYPE> void PowerBasisPolynomial<DATA_TYPE>::removeLeadingZeros()
{
    while (fuzzyEquals(this->_coeffs.back(), 0.0) && this->_coeffs.size() > 1) {
        this->_coeffs.pop_back();
    }
}

}  // namespace maths
}  // namespace robotics
