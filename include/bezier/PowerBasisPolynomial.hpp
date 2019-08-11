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

    friend std::ostream& operator<<(std::ostream& os, const PowerBasisPolynomial& polynomial)
    {
        assert(polynomial.degree() >= 0);

        os << "Polynomial function: \n";
        for (size_t i = 0; i < polynomial.coeffs().size(); ++i) {
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

template <typename DATA_TYPE> void PowerBasisPolynomial<DATA_TYPE>::removeLeadingZeros()
{
    while (fuzzyEquals(this->_coeffs.back(), 0.0) && this->_coeffs.size() > 1) {
        this->_coeffs.pop_back();
    }
}

}  // namespace maths
}  // namespace robotics
