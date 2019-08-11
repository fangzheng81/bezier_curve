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
    std::vector<double> coeffs{10.5, 4.7, 9.2, 0};

    robotics::maths::PowerBasisPolynomial<double> polynomial(coeffs);


    std::cout << polynomial << "\n";
    return 0;
}
