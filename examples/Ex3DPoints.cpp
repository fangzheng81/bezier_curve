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

#include <bezier/Bezier.hpp>

#ifndef WITH_VISUALIZATION
#define ENABLE_VISUALIZATION 1
#endif  // WITH_VISUALIZATION

#if ENABLE_VISUALIZATION
#include <matplotlib_cpp/MatplotlibCpp.hpp>
#endif  // ENABLE_VISUALIZATION

int main(int argc, char* argv[])
{
    using Bezier = robotics::Bezier<double, 3>;
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

    Bezier::VecPointType pV{p1, p2, p3, p4, p5, p6, p7, p8, p9, p10};

    Bezier::Ptr bezier = std::make_shared<Bezier>(9, pV);
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
        std::cout << p[0] << "," << p[1] << "," << p[2] << "\n";
    }

    std::cout << "Trajectory: \n";
    Bezier::VecPointType trajectory = bezier->trajectory(20);
    for (const auto& p : trajectory) {
        std::cout << p[0] << "," << p[1] << "," << p[2] << "\n";
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

    const auto xyzS = bezier->extractDataEachAxis(pV);

    mpllib.initializeAxes3D();

    mpllib.plotAxes3D(
        xyzS[0], xyzS[1], xyzS[2],
        {
            {"label", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("control points")},
            {"color", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("g")},
            {"linestyle", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("--")},
        });
    mpllib.scatterAxes3D(xyzS[0], xyzS[1], xyzS[2]);

    const auto xyzTrajectoryS = bezier->extractDataEachAxis(trajectory);

    mpllib.plotAxes3D(
        xyzTrajectoryS[0], xyzTrajectoryS[1], xyzTrajectoryS[2],
        {
            {"label", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("approximated bezier curve")},
            {"color", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("r")},
            {"linestyle", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("--")},
        });

    mpllib.legend();

    mpllib.savefig("3dpoints.png");

    mpllib.show();
#endif  // ENABLE_VISUALIZATION

    return 0;
}
