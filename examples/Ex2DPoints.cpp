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
    using Bezier = robotics::Bezier<5, double, 2>;
    Bezier::PointType p1(1, 2);
    Bezier::PointType p2(5, 7);
    Bezier::PointType p3(5, 11);
    Bezier::PointType p4(19, 20);
    Bezier::PointType p5(22, 23);
    Bezier::PointType p6(26, 27);

    Bezier::VecPointType pV(1, p1);
    pV.emplace_back(p2);
    pV.emplace_back(p3);
    pV.emplace_back(p4);
    pV.emplace_back(p5);
    pV.emplace_back(p6);

    Bezier::Ptr bezier = std::make_shared<Bezier>(pV);
    auto coeffV = bezier->BINOMIAL_COEFFS;

    auto t = bezier->controlPoints();
    auto binomialCoeffs_0 = robotics::maths::binomialCoeffs(2);

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
    Bezier::VecPointType trajectory = bezier->trajectory(5);
    for (const auto& p : trajectory) {
        std::cout << p[0] << "," << p[1] << "\n";
    }

    return 0;
}
