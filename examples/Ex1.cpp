/**
 * @file    Ex1.cpp
 *
 * @brief   test basic function of Bezier Class
 *
 * @author  btran
 *
 * @date    2019-06-19
 *
 * Copyright (c) organization
 *
 */

#include <bezier/Bezier.hpp>
#include <iostream>
#include <memory>

int main(int argc, char* argv[])
{
    typedef robotics::DefaultVector<double, 3> DefaultVector;
    using Bezier = robotics::Bezier<5, double, 3, DefaultVector>;
    DefaultVector p1(1, 2, 3);
    DefaultVector p2(5, 7, 3);

    Bezier::VecPointType pV(5, p1);
    pV.emplace_back(p2);

    Bezier::Ptr b = std::make_shared<Bezier>(pV);
    auto coeffV = b->BINOMIAL_COEFFS;

    auto t = b->controlPoints();

    auto temp = t[0][0];

    auto p = b->operator()(0.9);

    std::cout << p << "\n";

    return 0;
}
