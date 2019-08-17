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

#include <iostream>
#include <memory>
#include <string>

#include <bezier/Bezier.hpp>

#ifndef WITH_VISUALIZATION
#define ENABLE_VISUALIZATION 1
#endif  // WITH_VISUALIZATION

#if ENABLE_VISUALIZATION
#include <matplotlib_cpp/MatplotlibCpp.hpp>
#endif  // ENABLE_VISUALIZATION

int main(int argc, char* argv[])
{
    using Bezier = robotics::Bezier<double, 2>;
    Bezier::PointType p1(10.83, 6.44);
    Bezier::PointType p2(27.99, 9.75);
    Bezier::PointType p3(43.91, 14.3);
    Bezier::PointType p4(67.48, 24.84);
    Bezier::PointType p5(75.34, 46.97);
    Bezier::PointType p6(65.33, 86.25);

    Bezier::VecPointType pV{p1, p2, p3, p4, p5, p6};

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

#if ENABLE_VISUALIZATION
    // import modules of matplotlib library
    pe::vis::Matplotlib mpllib;

    // check if the modules are imported successully or not
    if (!mpllib.imported()) {
        std::cout << "Failed to import matplotlib library\n";
        exit(EXIT_FAILURE);
    }

    const auto xyS = bezier->extractDataEachAxis(pV);
    const auto xyTrajectoryS = bezier->extractDataEachAxis(trajectory);

    mpllib.plot(xyS[0], xyS[1],
                {
                    {"label", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("control points")},
                    {"color", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("g")},
                    {"linestyle", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("--")},
                });

    mpllib.scatter(xyS[0], xyS[1]);

    mpllib.plot(xyTrajectoryS[0], xyTrajectoryS[1],
                {
                    {"label", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("approximated bezier curve")},
                    {"color", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("r")},
                    {"linestyle", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("--")},
                });

    mpllib.legend();

    mpllib.grid();

    mpllib.savefig("2dpoints.png");

    mpllib.show();

#endif  // ENABLE_VISUALIZATION
    return 0;
}
