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

#ifdef BUILD_VISUALIZATION
#include <vtkSmartPointer.h>
#include <vtkVersion.h>

#include <vtkChartXY.h>
#include <vtkContextScene.h>
#include <vtkContextView.h>
#include <vtkFloatArray.h>
#include <vtkPlotPoints.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTable.h>
#endif

int main(int argc, char* argv[])
{
    using DefaultVector = robotics::DefaultVector<double, 2>;
    using Bezier = robotics::Bezier<5, double, 2, DefaultVector>;
    DefaultVector p1(1, 2);
    DefaultVector p2(5, 7);
    DefaultVector p3(5, 11);
    DefaultVector p4(19, 20);
    DefaultVector p5(22, 23);
    DefaultVector p6(26, 27);

    Bezier::VecPointType pV(1, p1);
    pV.emplace_back(p2);
    pV.emplace_back(p3);
    pV.emplace_back(p4);
    pV.emplace_back(p5);
    pV.emplace_back(p6);

    Bezier::Ptr b = std::make_shared<Bezier>(pV);
    auto coeffV = b->BINOMIAL_COEFFS;

    auto t = b->controlPoints();
    auto binomialCoeffs_0 = robotics::maths::binomialCoeffs(2);

    const Bezier::Tangent tan = b->tangent(0.7);
    const Bezier::Normal nor = b->normal(0.7);
    double curvature = b->curvature(0.7);

    std::cout << "tangent vector"
              << "\n";
    std::cout << tan << "\n";

    std::cout << "normal vector"
              << "\n";
    std::cout << nor << "\n";

    std::cout << "dot product: " << tan.dot(nor) << "\n";

    std::cout << "curvature: " << curvature << "\n";

    // std::cout << "------------------------------------------------"
    //           << "\n";

    // std::cout << "Original control points"
    //           << "\n";
    // for (const auto& p : b->controlPoints()) {
    //     std::cout << p[0] << "," << p[1] << "," << p[2] << "\n";
    // }

    // std::cout << "Trajectory: "
    //           << "\n";
    // Bezier::VecPointType trajectory = b->trajectory(5);
    // std::cout << "------------------------------------------------"
    //           << "\n";
    // for (auto& p : trajectory) {
    //     std::cout << p[0] << "," << p[1] << "," << p[2] << "\n";
    // }

    // std::cout << "------------------------------------------------"
    //           << "\n";

    return 0;
}
