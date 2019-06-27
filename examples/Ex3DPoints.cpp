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
    using Bezier = robotics::Bezier<9, double, 3>;
    Bezier::PointType p1(1, 5, 2);
    Bezier::PointType p2(2, 6, 3);
    Bezier::PointType p3(3, 2, 3);
    Bezier::PointType p4(4, 3, 3);
    Bezier::PointType p5(5, 13, 5);
    Bezier::PointType p6(6, 4, 7);
    Bezier::PointType p7(7, 1, 9);
    Bezier::PointType p8(8, 2, 11);
    Bezier::PointType p9(9, 4, 9);
    Bezier::PointType p10(10, 8, 10);

    Bezier::VecPointType pV;
    pV.reserve(10);

    pV.emplace_back(p1);
    pV.emplace_back(p2);
    pV.emplace_back(p3);
    pV.emplace_back(p4);
    pV.emplace_back(p5);
    pV.emplace_back(p6);
    pV.emplace_back(p7);
    pV.emplace_back(p8);
    pV.emplace_back(p9);
    pV.emplace_back(p10);

    Bezier::Ptr bezier = std::make_shared<Bezier>(pV);
    auto coeffV = bezier->BINOMIAL_COEFFS;

    const Bezier::Tangent tan = bezier->tangent(0.7);
    const Bezier::Normal nor = bezier->normal(0.7);
    double curvature = bezier->curvature(0.7);

    std::cout << "tangent vector: \n" << tan << "\n";
    std::cout << "normal vector: \n" << nor << "\n";
    std::cout << "dot product: " << tan.dot(nor) << "\n";
    std::cout << "curvature: " << curvature << "\n";

    std::cout << "Original control points: \n";
    for (const auto& p : bezier->controlPoints()) {
        std::cout << p[0] << "," << p[1] << "," << p[2] << "\n";
    }

    std::cout << "Trajectory: \n";
    Bezier::VecPointType trajectory = bezier->trajectory(20);
    for (const auto& p : trajectory) {
        std::cout << p[0] << "," << p[1] << "," << p[2] << "\n";
    }

    std::cout << *bezier << "\n";

    return 0;
}
