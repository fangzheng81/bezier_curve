/**
 * @file    ExUserDefinedPointType.cpp
 *
 * @brief   test bezier curve with user-defined point type
 *
 * @author  bt
 *
 * @date    2019-07-20
 *
 * Copyright (c) organization
 *
 */

#include <bezier/Bezier.hpp>
#include <iostream>
#include <memory>

class NewPointType : public Eigen::Vector3d
{
 public:
    NewPointType() : Eigen::Vector3d()
    {
    }

    template <typename OtherDerived>
    explicit NewPointType(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Vector3d(other)
    {
    }

    template <typename OtherDerived> NewPointType& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Eigen::Vector3d::operator=(other);
        return *this;
    }

    NewPointType(const Eigen::Vector3d& other, double otherVariable)
        : Eigen::Vector3d(other), _otherVariable(otherVariable)
    {
    }

    explicit NewPointType(double* v) : Eigen::Vector3d(v)
    {
    }

 private:
    double _otherVariable;
};

int main(int argc, char* argv[])
{
    using EigenBezier = robotics::Bezier<9, double, 3>;
    EigenBezier::VecPointType eigenPV{{1, 2, 5}, {2, 6, 3}, {3, 2, 3},  {4, 3, 3}, {5, 13, 5},
                                      {6, 4, 7}, {7, 1, 9}, {8, 2, 11}, {9, 4, 9}, {10, 8, 10}};

    using Bezier = robotics::Bezier<9, double, 3, NewPointType>;
    Bezier::VecPointType pV;

    std::transform(eigenPV.begin(), eigenPV.end(), std::back_inserter(pV),
                   [](const EigenBezier::PointType& p) { return Bezier::PointType(p, 10); });

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
