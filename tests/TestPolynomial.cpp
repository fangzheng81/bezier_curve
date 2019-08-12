/**
 * @file    TestPolynomial.cpp
 *
 * @brief   test polynomial
 *
 * @author  btran
 *
 * @date    2019-08-11
 *
 * Copyright (c) organization
 *
 */

#include "gtest/gtest.h"
#include <bezier/PowerBasisPolynomial.hpp>
#include <ctime>

class TestPolynomial : public ::testing::Test
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

TEST_F(TestPolynomial, TestInitialization)
{
    using Polynomial = robotics::maths::PowerBasisPolynomial<double>;

    {
        std::vector<double> coeffs{1.9, 3.4, 5.7};
        Polynomial polynomial(coeffs);
        EXPECT_TRUE(polynomial.degree() == coeffs.size() - 1);
    }

    {
        std::vector<double> coeffs{0.0};
        Polynomial polynomial(coeffs);
        EXPECT_TRUE(polynomial.degree() == coeffs.size() - 1);
    }

    {
        std::vector<double> coeffs{1.0};
        Polynomial polynomial(coeffs);
        EXPECT_TRUE(polynomial.degree() == coeffs.size() - 1);
    }
}
