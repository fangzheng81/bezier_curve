/**
 * @file    SturmSequence.hpp
 *
 * @author  btran
 *
 * @date    2019-08-12
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <cmath>
#include <functional>
#include <vector>

#include "bezier/PowerBasisPolynomial1D.hpp"

namespace robotics
{
namespace maths
{
/**
 *  @brief Sturm Sequence is a sequences of polynomial generated for root separation of a target polynomial
 *
 *  Given the input power basis polynomial Q(t)
 *  The following sturm sequence (P_i(t))will be generated:
 *  P_0(t) = Q(t);
 *  P_1(t) = Q'(t);
 *  P_n(t) = - P_(n-1)(t) % P_(n-2)(t)
 */
template <typename DATA_TYPE = double> class SturmSequence
{
 public:
    //! tolerant floating point precision
    static constexpr DATA_TYPE TOLERANCE = 10e-7;

    /**
     *  @brief constructing sturm sequence with the original polynomial Q(t)
     *
     *  Q(t) will be divided by gcd of (Q(t), Q'(t)) to obtain a square-free polynomial
     *  the targeted sequences will be generated in this constructor
     *
     */
    explicit SturmSequence(const PowerBasisPolynomial1D<DATA_TYPE>& polynomial, const DATA_TYPE tolerance = TOLERANCE);

    /**
     *  @brief constant getter of the generated sturm sequences
     *
     */
    const std::vector<PowerBasisPolynomial1D<DATA_TYPE>>& sturmSeqs() const;

    /**
     *  @brief count number of sign changes at value t
     *
     */
    int countSturmSignChanges(DATA_TYPE t) const;

    /**
     *  @brief solver to separate and find roots at this interval
     *
     *  @return the vector of found roots
     */
    std::vector<DATA_TYPE> solve(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const;

    /**
     *  @brief solver to separate and find minima roots at this interval
     *
     *  @return the vector of found roots
     */
    std::vector<DATA_TYPE> solveLocalMinium(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const;

    /**
     *  @brief solver to find root at this interval
     *
     *  @return the root at this interval, NAN if none found
     */
    DATA_TYPE solveBisection(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const;

    /**
     *  @brief solver to find root, that would be local minium, at this interval
     *
     *  @return the root at this interval, NAN if none found
     */
    DATA_TYPE solveBisectionLocalMinium(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const;

 private:
    /**
     *  @brief the utility struct to store an interval [lowerBound, upperBound]
     *
     */
    struct Interval {
        DATA_TYPE lowerBound;
        DATA_TYPE upperBound;
    };

    /**
     *  @brief utility struct to store an interval with id for identification
     */
    struct SturmInterval : public Interval {
        SturmInterval(const DATA_TYPE lowerBoundVal, const DATA_TYPE upperBoundVal, const int lowerBoundSign,
                      const int upperBoundSign, const int id, const int expectedRoots)
            : lowerBoundSign(lowerBoundSign), upperBoundSign(upperBoundSign), id(id), expectedRoots(expectedRoots)
        {
            if (lowerBoundVal > upperBoundVal) {
                throw std::runtime_error("lower bound must be less than upper bound");
            }
            this->lowerBound = lowerBoundVal;
            this->upperBound = upperBoundVal;
        }

        //! number of sign changes at lower bound
        int lowerBoundSign;

        //! number of sign changes at upper bound
        int upperBoundSign;

        //! id of the interval
        int id;

        //! expected number of roots held in this interval
        int expectedRoots;
    };

    /**
     *  @brief generate Sturm sequences
     *
     *  @return the generated Sturm sequences
     */
    std::vector<PowerBasisPolynomial1D<DATA_TYPE>> generateSturmSequences() const;

    /**
     *  @brief utility function to separate roots into interval
     *
     */
    std::pair<std::vector<Interval>, int> separateRoots(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const;

    /**
     *  @brief utility function to find roots using provided solver
     *
     */
    std::vector<DATA_TYPE> solverUtil(const DATA_TYPE lowerBound, const DATA_TYPE upperBound,
                                      const std::function<DATA_TYPE(const DATA_TYPE, const DATA_TYPE)>& solver) const;

 private:
    //! the original polynomial. Note that the original polynomial will be transformed into a square-free polynomial.
    PowerBasisPolynomial1D<DATA_TYPE> _polynomial;

    //! member variable to hold the generated Sturm sequences
    std::vector<PowerBasisPolynomial1D<DATA_TYPE>> _sturmSeqs;

    //! max division estimated from the tolerant precision
    int _maxDivisions;

    //! tolerant floating point precision
    DATA_TYPE _tolerance;
};

template <typename DATA_TYPE>
SturmSequence<DATA_TYPE>::SturmSequence(const PowerBasisPolynomial1D<DATA_TYPE>& polynomial, const DATA_TYPE tolerance)
    : _polynomial(polynomial.monicized()), _tolerance(tolerance)
{
    const PowerBasisPolynomial1D<DATA_TYPE> P0 = polynomial;
    const PowerBasisPolynomial1D<DATA_TYPE> P1 = polynomial.derivative();
    const PowerBasisPolynomial1D<DATA_TYPE> gcdP = PowerBasisPolynomial1D<DATA_TYPE>::gcd(P0, P1);

    if (gcdP.degree() != 0) {
        this->_polynomial = (P0.divide(gcdP).first).monicized();
    }

    this->_sturmSeqs = this->generateSturmSequences();
    this->_maxDivisions = 1 + static_cast<int>(std::log2(1.f / this->_tolerance));
}

template <typename DATA_TYPE>
const std::vector<PowerBasisPolynomial1D<DATA_TYPE>>& SturmSequence<DATA_TYPE>::sturmSeqs() const
{
    return this->_sturmSeqs;
}

template <typename DATA_TYPE> int SturmSequence<DATA_TYPE>::countSturmSignChanges(DATA_TYPE t) const
{
    int signChanges = 0;

    uint32_t previousSign = sgn(this->_polynomial(t));

    for (auto it = this->_sturmSeqs.cbegin() + 1; it != this->_sturmSeqs.cend(); ++it) {
        uint32_t currentSign = sgn(it->operator()(t));
        signChanges += (previousSign ^ currentSign) >> 31;
        previousSign = currentSign;
    }

    return signChanges;
}

template <typename DATA_TYPE>
std::vector<DATA_TYPE> SturmSequence<DATA_TYPE>::solve(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const
{
    return solverUtil(lowerBound, upperBound, [this](const DATA_TYPE lowerBound, const DATA_TYPE upperBound) {
        return this->solveBisection(lowerBound, upperBound);
    });
}

template <typename DATA_TYPE>
std::vector<DATA_TYPE> SturmSequence<DATA_TYPE>::solveLocalMinium(const DATA_TYPE lowerBound,
                                                                  const DATA_TYPE upperBound) const
{
    return solverUtil(lowerBound, upperBound, [this](const DATA_TYPE lowerBound, const DATA_TYPE upperBound) {
        return this->solveBisectionLocalMinium(lowerBound, upperBound);
    });
}

template <typename DATA_TYPE>
DATA_TYPE SturmSequence<DATA_TYPE>::solveBisection(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const
{
    DATA_TYPE lowerVal = this->_polynomial(lowerBound);
    DATA_TYPE upperVal = this->_polynomial(upperBound);

    if (!(sgn(lowerVal) ^ sgn(upperVal))) {
        return NAN;
    }

    const int maxIterations = this->_maxDivisions;
    DATA_TYPE bisectionMin = lowerBound;
    DATA_TYPE bisectionMax = upperBound;

    DATA_TYPE mid;
    int i = 0;
    while ((bisectionMax - bisectionMin) >= this->_tolerance && i < maxIterations) {
        mid = (bisectionMax + bisectionMin) / 2.f;
        DATA_TYPE midVal = this->_polynomial(mid);

        if (std::fabs(midVal) <= this->_tolerance) {
            return mid;
        }

        if (sgn(lowerVal) ^ sgn(midVal)) {
            bisectionMax = mid;
        } else {
            lowerVal = midVal;
            bisectionMin = mid;
        }
        ++i;
    }

    return bisectionMin;
}

template <typename DATA_TYPE>
DATA_TYPE SturmSequence<DATA_TYPE>::solveBisectionLocalMinium(const DATA_TYPE lowerBound,
                                                              const DATA_TYPE upperBound) const
{
    DATA_TYPE lowerVal = this->_polynomial(lowerBound);
    DATA_TYPE upperVal = this->_polynomial(upperBound);

    if (lowerVal > 0) {
        return NAN;
    }

    if (upperVal < 0) {
        return NAN;
    }

    return this->solveBisection(lowerBound, upperBound);
}

template <typename DATA_TYPE>
std::vector<PowerBasisPolynomial1D<DATA_TYPE>> SturmSequence<DATA_TYPE>::generateSturmSequences() const
{
    assert(this->_polynomial.coeffs().size() > 0);

    if (this->_polynomial.coeffs().size() == 1) {
        return {this->_polynomial};
    }

    std::vector<PowerBasisPolynomial1D<DATA_TYPE>> result;
    result.emplace_back(this->_polynomial);
    result.emplace_back(this->_polynomial.derivative());

    PowerBasisPolynomial1D<DATA_TYPE> remainderPoly =
        (std::prev(std::prev(result.end()))->divide(result.back())).second;

    while (!remainderPoly.isZero()) {
        result.emplace_back(remainderPoly.multiply(-1), true);
        remainderPoly = (std::prev(std::prev(result.end()))->divide(result.back())).second;
    }

    return result;
}

template <typename DATA_TYPE>
std::pair<std::vector<typename SturmSequence<DATA_TYPE>::Interval>, int>
SturmSequence<DATA_TYPE>::separateRoots(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const
{
    std::vector<SturmInterval> intervalStorage;
    intervalStorage.reserve(this->_maxDivisions);

    int lowerBoundSign = this->countSturmSignChanges(lowerBound);
    int upperBoundSign = this->countSturmSignChanges(upperBound);
    const int totalRoots = lowerBoundSign - upperBoundSign;
    int id = 0;

    intervalStorage.emplace_back(lowerBound, upperBound, lowerBoundSign, upperBoundSign, id, totalRoots);
    id++;

    std::vector<Interval> rootIntervals(this->_polynomial.degree());
    int foundRoots = 0;

    while (!intervalStorage.empty() && foundRoots != totalRoots) {
        SturmInterval curSI = intervalStorage.back();
        intervalStorage.pop_back();

        int numRoots = curSI.lowerBoundSign - curSI.upperBoundSign;

        if (numRoots <= 0) {
            if (!intervalStorage.empty() && intervalStorage.back().id == curSI.id) {
                curSI = intervalStorage.back();
                intervalStorage.pop_back();
                numRoots = curSI.expectedRoots;
            } else {
                continue;
            }
        }

        if (numRoots == curSI.expectedRoots && !intervalStorage.empty() && intervalStorage.back().id == curSI.id) {
            intervalStorage.pop_back();
        } else if (numRoots == curSI.expectedRoots - 1 && !intervalStorage.empty() &&
                   intervalStorage.back().id == curSI.id) {
            rootIntervals[foundRoots++] = intervalStorage.back();
            intervalStorage.pop_back();
        }

        if (numRoots == 1) {
            rootIntervals[foundRoots++] = curSI;
        } else {
            DATA_TYPE mid = (curSI.lowerBound + curSI.upperBound) / 2.f;
            if (mid - curSI.lowerBound <= this->_tolerance) {
                rootIntervals[foundRoots++] = curSI;
            } else {
                const int signMid = this->countSturmSignChanges(mid);

                intervalStorage.emplace_back(curSI.lowerBound, mid, curSI.lowerBoundSign, signMid, id, numRoots);
                intervalStorage.emplace_back(mid, curSI.upperBound, signMid, curSI.upperBoundSign, id, numRoots);

                id++;
            }
        }
    }

    return std::make_pair(rootIntervals, foundRoots);
}

template <typename DATA_TYPE>
std::vector<DATA_TYPE>
SturmSequence<DATA_TYPE>::solverUtil(const DATA_TYPE lowerBound, const DATA_TYPE upperBound,
                                     const std::function<DATA_TYPE(const DATA_TYPE, const DATA_TYPE)>& solver) const
{
    auto rootIntervalPairs = this->separateRoots(lowerBound, upperBound);
    const int foundRoots = rootIntervalPairs.second;
    const std::vector<Interval>& rootIntervals = rootIntervalPairs.first;

    std::vector<DATA_TYPE> realRoots;
    realRoots.reserve(foundRoots);

    for (size_t i = 0; i < foundRoots; ++i) {
        auto root = solver(rootIntervals[i].lowerBound, rootIntervals[i].upperBound);
        if (!std::isnan(root)) {
            realRoots.emplace_back(root);
        }
    }

    return realRoots;
}

}  // namespace maths
}  // namespace robotics
