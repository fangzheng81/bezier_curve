/**
 * @file    ExApproximateLength.cpp
 *
 * @author  bt
 *
 * @date    2019-09-20
 *
 * Copyright (c) organization
 *
 */

#include <chrono>

#include <bezier/Bezier.hpp>

int main(int argc, char* argv[])
{
    using EigenBezier = robotics::Bezier<double, 3>;
    EigenBezier::VecPointType eigenPV{{1, 2, 5}, {2, 6, 3}, {3, 2, 3},  {4, 3, 3}, {5, 13, 5},
                                      {6, 4, 7}, {7, 1, 9}, {8, 2, 11}, {9, 4, 9}, {10, 8, 10}};

    EigenBezier bezier(eigenPV);

    double appLength1, appLength2;

    {
        auto start = std::chrono::high_resolution_clock::now();

        EigenBezier::VecPointType trajectory = bezier.trajectory(100);
        double appLength2 = bezier.approximateLength(trajectory);

        auto end = std::chrono::high_resolution_clock::now();

        double executedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

        // change into sec
        executedTime *= 1e-9;

        std::cout << "appLength2: " << appLength2 << "\n";
        std::cout << "[2] Time taken by program is : " << std::fixed << executedTime << std::setprecision(6)
                  << " [sec]\n";
    }

    return 0;
}
