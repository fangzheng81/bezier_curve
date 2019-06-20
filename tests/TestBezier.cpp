/**
 * @file    TestBezier.cpp
 *
 * @brief   Test Basic Function of Bezier
 *
 * @author  btran
 *
 * @date    2019-06-20
 *
 * Copyright (c) organization
 *
 */

#include "gtest/gtest.h"
#include <bezier/Bezier.hpp>
#include <ctime>

class TestBezier : public ::testing::Test
{
 protected:
    void SetUp() override
    {
        start_time_ = time(nullptr);
    }

    void TearDown() override
    {
        const time_t end_time = time(nullptr);

        // expect test time less than 5 sec
        EXPECT_LE(end_time - start_time_, 5);
    }

    time_t start_time_;
};

TEST_F(TestBezier, TestInitialization)
{
    using Bezier5d = robotics::Bezier<5, double, 3>;
    using Bezier10d = robotics::Bezier<10, double, 3>;
    using Bezier15d = robotics::Bezier<15, double, 3>;
    using Bezier20d = robotics::Bezier<20, double, 2>;
    using Bezier100d = robotics::Bezier<100, double, 3>;

    {
        ASSERT_EQ(Bezier5d().BINOMIAL_COEFFS.size(), 6);
        ASSERT_EQ(Bezier100d().BINOMIAL_COEFFS.size(), 101);
    }

    {
        const std::vector<double> binomialCoeffs = Bezier5d().BINOMIAL_COEFFS;
        std::vector<double> expectedBinomialCoeffs{1, 5, 10, 10, 5, 1};

        EXPECT_EQ(binomialCoeffs, expectedBinomialCoeffs);
    }

    {
        const std::vector<double> binomialCoeffs = Bezier10d().BINOMIAL_COEFFS;
        std::vector<double> expectedBinomialCoeffs{1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1};

        EXPECT_EQ(binomialCoeffs, expectedBinomialCoeffs);
    }

    {
        const std::vector<double> binomialCoeffs = Bezier15d().BINOMIAL_COEFFS;
        std::vector<double> expectedBinomialCoeffs{1,    15,   105,  455,  1365, 3003, 5005, 6435,
                                                   6435, 5005, 3003, 1365, 455,  105,  15,   1};

        EXPECT_EQ(binomialCoeffs, expectedBinomialCoeffs);
    }

    {
        const std::vector<double> binomialCoeffs = Bezier20d().BINOMIAL_COEFFS;
        std::vector<double> expectedBinomialCoeffs{1,     20,     190,    1140,   4845,   15504,  38760,
                                                   77520, 125970, 167960, 184756, 167960, 125970, 77520,
                                                   38760, 15504,  4845,   1140,   190,    20,     1};

        EXPECT_EQ(binomialCoeffs, expectedBinomialCoeffs);
    }
}

TEST_F(TestBezier, TestControlPoints)
{
    using Bezier3d = robotics::Bezier<3, double, 2>;
    Bezier3d::VecPointType v({{120, 160}, {35, 200}, {220, 260}, {220, 40}});
    Bezier3d bezier3d(v);
    double t;

    {
        t = 0;
        Bezier3d::PointType p = bezier3d(t);
        Bezier3d::PointType expectedP;
        expectedP << 120 + 0.00001, 160;
        EXPECT_TRUE(Bezier3d::fuzzyEquals(p, expectedP));
    }

    {
        t = 1;
        Bezier3d::PointType p = bezier3d(t);
        Bezier3d::PointType expectedP;
        expectedP << 220 + 0.00001, 160 + 0.001;
        EXPECT_FALSE(Bezier3d::fuzzyEquals(p, expectedP));
    }

    {
        t = 0.25;
        Bezier3d::PointType p = bezier3d(t);
        Bezier3d::PointType expectedP;
        expectedP << 99.765625, 189.0625;
        EXPECT_TRUE(Bezier3d::fuzzyEquals(p, expectedP));
    }

    {
        t = 0.5;
        Bezier3d::PointType p = bezier3d(t);
        Bezier3d::PointType expectedP;
        expectedP << 138.125, 197.5;
        EXPECT_TRUE(Bezier3d::fuzzyEquals(p, expectedP));
    }

    {
        t = 0.75;
        Bezier3d::PointType p = bezier3d(t);
        Bezier3d::PointType expectedP;
        expectedP << 192.421875, 157.1875;
        EXPECT_TRUE(Bezier3d::fuzzyEquals(p, expectedP));
    }

    {
        t = -0.35;
        Bezier3d::PointType p = bezier3d(t);
        Bezier3d::PointType expectedP;
        expectedP << 327.983124, 138.212509;
        EXPECT_TRUE(Bezier3d::fuzzyEquals(p, expectedP));
    }

    {
        t = 1.5;
        Bezier3d::PointType p = bezier3d(t);
        Bezier3d::PointType expectedP;
        expectedP << 24.375, -537.5;
        EXPECT_TRUE(Bezier3d::fuzzyEquals(p, expectedP));
    }
}
