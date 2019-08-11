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

#include <deque>

namespace robotics
{
namespace maths
{
template <typename DATA_TYPE = double> class PowerBasisPolynomial
{
 public:
    PowerBasisPolynomial();
    virtual ~PowerBasisPolynomial();

    const size_t degree() const;

 private:
    std::deque<DATA_TYPE> _coeffs;
};

template <typename DATA_TYPE> const size_t PowerBasisPolynomial<DATA_TYPE>::degree() const
{
    return this->_coeffs.size() - 1;
}

}  // namespace maths
}  // namespace robotics
