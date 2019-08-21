/**
 * @file    ExBoundingBox.cpp
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

enum MPL_PATH { STOP = 0, MOVETO = 1, LINETO = 2, CURVE3 = 3, CURVE4 = 4, CLOSEPOLY = 5 };

int main(int argc, char* argv[])
{
    using Bezier = robotics::Bezier<double, 2>;
    const Bezier::VecPointType pV{{1.1, -0.6}, {3.2, 0.9}, {4.9, 2.4}, {5.9, 3.8}, {7.2, 2.13}};

    Bezier bezier(4, pV);

    const Bezier::VecPointType trajectory = bezier.trajectory(20);

    const Bezier::BoundingBox bbox = bezier.estimateBoundingBox();

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

    mpllib.axes(5, 5);
    std::vector<std::array<double, 2>> vertices = {
        {bbox[0].x(), bbox[0].y()}, {bbox[1].x(), bbox[0].y()}, {bbox[1].x(), bbox[1].y()},
        {bbox[0].x(), bbox[1].y()}, {bbox[0].x(), bbox[0].y()},
    };

    std::vector<int> codes = {MOVETO, LINETO, LINETO, LINETO, CLOSEPOLY};

    mpllib.add_patch(vertices, codes,
                    {
                        {"facecolor", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("None")},
                        {"edgecolor", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("green")},
                        {"lw", pe::vis::Matplotlib::createAnyBaseMapData<int>(2)},
                    });

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

    mpllib.savefig("boundingbox.png");

    mpllib.show();

#endif  // ENABLE_VISUALIZATION
    return EXIT_SUCCESS;
}
