/**
 * @file    ExPolynomial.cpp
 *
 * @brief   test polnomial
 *
 * @author  btran
 *
 * @date    2019-08-11
 *
 * Copyright (c) organization
 *
 */

#include <bezier/PowerBasisPolynomial.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    std::vector<double> coeffs{1, -1, 5, -2};
    std::vector<double> otherCoeffs{-2, 3};

    robotics::maths::PowerBasisPolynomial<double> polynomial(coeffs);
    robotics::maths::PowerBasisPolynomial<double> otherPolynomial(otherCoeffs);

    auto sum = polynomial.add(otherPolynomial);
    auto multiplication = polynomial.multiply(otherPolynomial);

    std::cout << multiplication << "\n";
    std::cout << multiplication.derivative() << "\n";

    robotics::maths::PowerBasisPolynomial<double> polynomial0(
        std::vector<double>{8.5, 3.45, 0.0, 0.0, 5.7, 4.9, 0.1, 10});

    std::cout << polynomial0 << "\n";

    return 0;
}
