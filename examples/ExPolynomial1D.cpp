/**
 * @file    ExPolynomial1D.cpp
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

#include <bezier/PowerBasisPolynomial1D.hpp>
#include <bezier/SturmSequence.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    using Polynomial = robotics::maths::PowerBasisPolynomial1D<double>;

    // std::vector<double> coeffs{1, -1, 5, -2};
    // std::vector<double> otherCoeffs{-2, 3};

    // Polynomial polynomial(coeffs);
    // Polynomial otherPolynomial(otherCoeffs);

    // auto sum = polynomial.add(otherPolynomial);
    // auto multiplication = polynomial.multiply(otherPolynomial);

    // std::cout << multiplication << "\n";
    // std::cout << multiplication.derivative() << "\n";

    // Polynomial polynomial0(std::vector<double>{8.5, 3.45, 0.0, 0.0, 5.7, 4.9, 0.1, 10});

    // Polynomial u(std::vector<double>{8, 10, -5, 3});
    // Polynomial v(std::vector<double>{-3, 2, 1});
    // auto qr = u.divide(v);

    // std::cout << qr.first << "\n";
    // std::cout << qr.second << "\n";

    // robotics::maths::SturmSequence<double> ss(polynomial0);

    // for (const auto& seq : ss.sturmSeqs()) {
    //     std::cout << seq << "\n";
    // }

    Polynomial polynomial(std::vector<double>{8, -20, 14, 1, -4, 1});

    robotics::maths::SturmSequence<double> ss(polynomial);
    for (const auto& seq : ss.sturmSeqs()) {
        std::cout << seq << "\n";
    }

    auto roots = ss.solveLocalMinium(-10.0, 10.0);
    for (const double root : roots) {
        std::cout << root << "\n";
    }

    // Polynomial temp(std::vector<double>{-1, 1});
    // Polynomial temp2(std::vector<double>{-2, 1});
    // Polynomial temp3(std::vector<double>{2, 1});

    // auto a = temp.multiply(temp).multiply(temp).multiply(temp);
    // auto b = temp.multiply(temp2).multiply(temp2);
    // auto c = a.multiply(b).multiply(temp3);

    // std::cout << c << "\n";

    return 0;
}
