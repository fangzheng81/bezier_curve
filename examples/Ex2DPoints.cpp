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
    using Bezier = robotics::Bezier<double, 2>;
    Bezier::PointType p1(10.83, 6.44);
    Bezier::PointType p2(27.99, 9.75);
    Bezier::PointType p3(43.91, 14.3);
    Bezier::PointType p4(67.48, 24.84);
    Bezier::PointType p5(75.34, 46.97);
    Bezier::PointType p6(65.33, 86.25);

    Bezier::VecPointType pV;
    pV.reserve(6);

    pV.emplace_back(p1);
    pV.emplace_back(p2);
    pV.emplace_back(p3);
    pV.emplace_back(p4);
    pV.emplace_back(p5);
    pV.emplace_back(p6);

    Bezier::Ptr bezier = std::make_shared<Bezier>(5, pV);
    auto coeffV = bezier->binomialCoeffs();

    const Bezier::Tangent tan = bezier->tangent(0.7);
    const Bezier::Normal nor = bezier->normal(0.7);
    double curvature = bezier->curvature(0.7);

    std::cout << "tangent vector: \n" << tan << "\n";
    std::cout << "normal vector: \n" << nor << "\n";
    std::cout << "dot product: " << tan.dot(nor) << "\n";
    std::cout << "curvature: " << curvature << "\n";

    std::cout << "Original control points: \n";
    for (const auto& p : bezier->controlPoints()) {
        std::cout << p[0] << "," << p[1] << "\n";
    }

    std::cout << "Trajectory: \n";
    Bezier::VecPointType trajectory = bezier->trajectory(20);
    for (const auto& p : trajectory) {
        std::cout << p[0] << "," << p[1] << "\n";
    }

    std::cout << *bezier << "\n";

    return 0;
}
