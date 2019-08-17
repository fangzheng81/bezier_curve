/**
 * @file    ClosestPointsToCurve.cpp
 *
 * @author  btran
 *
 * @date    2019-08-17
 *
 * Copyright (c) organization
 *
 */

#include <iostream>

#include <bezier/Bezier.hpp>
#include <bezier/SturmSequence.hpp>

#ifndef WITH_VISUALIZATION
#define ENABLE_VISUALIZATION 1
#endif  // WITH_VISUALIZATION

#if ENABLE_VISUALIZATION
#include <matplotlib_cpp/MatplotlibCpp.hpp>
#endif  // ENABLE_VISUALIZATION

int main(int argc, char* argv[])
{
    using Bezier = robotics::Bezier<double, 2>;
    const Bezier::VecPointType pV{{1.0, 0.0}, {3.0, 0.0}, {4.9, 2.4}, {5.9, 3.0}, {7.2, 3.0}};

    Bezier bezier(4, pV);

    const Bezier::VecPointType trajectory = bezier.trajectory(20);

    const Bezier::VecPointType outliners{{0.5, 0},   {2, 0.5},   {2.5, 0.1},  {2.5, 1.5}, {3.0, 0.0},
                                         {3.5, 1.9}, {4.8, 0.6}, {5.2, 1.25}, {6, 1.7}};
    Bezier::VecPointType closestPoints;
    closestPoints.reserve(outliners.size());

    for (const auto& outliner : outliners) {
        const double closestVal = bezier.closestPointToCurve(outliner);
        closestPoints.emplace_back(bezier(closestVal));
    }

#if ENABLE_VISUALIZATION
    // import modules of matplotlib library
    pe::vis::Matplotlib mpllib;

    // check if the modules are imported successully or not
    if (!mpllib.imported()) {
        std::cout << "Failed to import matplotlib library\n";
        exit(EXIT_FAILURE);
    }

    const auto xyS = bezier.extractDataEachAxis(pV);
    const auto xyTrajectoryS = bezier.extractDataEachAxis(trajectory);

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

    for (size_t i = 0; i < outliners.size(); ++i) {
        const auto outliner = outliners[i];
        const auto closestPoint = closestPoints[i];

        mpllib.plot(std::vector<double>{outliner.x(), closestPoint.x()},
                    std::vector<double>{outliner.y(), closestPoint.y()},
                    {
                        {"label", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("outliner to closest point")},
                        {"color", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("y")},
                    });

        mpllib.scatter(std::vector<double>{outliner.x(), closestPoint.x()},
                       std::vector<double>{outliner.y(), closestPoint.y()},
                       {
                           {"color", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("b")},
                       });
    }

    mpllib.legend();

    mpllib.grid();

    mpllib.savefig("pointprojection.png");

    mpllib.show();

#endif  // ENABLE_VISUALIZATION
    return EXIT_SUCCESS;
}
