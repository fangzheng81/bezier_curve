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

#include <cfloat>
#include <cmath>
#include <vector>

#include "bezier/PowerBasisPolynomial1D.hpp"

namespace robotics
{
namespace maths
{
template <typename DATA_TYPE = double> class SturmSequence
{
 public:
    explicit SturmSequence(const PowerBasisPolynomial1D<DATA_TYPE>& polynomial, const DATA_TYPE tolerance = 0.00001f);

    const std::vector<PowerBasisPolynomial1D<DATA_TYPE>>& sturmSeqs() const;

    int countSturmSignChanges(DATA_TYPE t) const;

    std::vector<DATA_TYPE> solveLocalMinium(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const;

    DATA_TYPE solveBisectionLocalMinium(const DATA_TYPE lowerBound, const DATA_TYPE upperBound) const;

 private:
    std::vector<PowerBasisPolynomial1D<DATA_TYPE>> generateSturmSequences() const;

 private:
    struct Interval {
        DATA_TYPE lowerBound;
        DATA_TYPE upperBound;
    };

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

        int lowerBoundSign;
        int upperBoundSign;
        int id;
        int expectedRoots;
    };

    PowerBasisPolynomial1D<DATA_TYPE> _polynomial;
    std::vector<PowerBasisPolynomial1D<DATA_TYPE>> _sturmSeqs;
    int _maxDivisions;
    DATA_TYPE _tolerance;
};

template <typename DATA_TYPE>
SturmSequence<DATA_TYPE>::SturmSequence(const PowerBasisPolynomial1D<DATA_TYPE>& polynomial, const DATA_TYPE tolerance)
    : _polynomial(polynomial), _tolerance(tolerance)
{
    this->_sturmSeqs = this->generateSturmSequences();
    this->_maxDivisions = 1 + static_cast<int>(log2(1.f / this->_tolerance));
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
        PowerBasisPolynomial1D<DATA_TYPE> curPoly = *it;
        uint32_t currentSign = sgn(curPoly(t));
        signChanges += (previousSign ^ currentSign) >> 31;
        previousSign = currentSign;
    }

    return signChanges;
}

template <typename DATA_TYPE>
std::vector<DATA_TYPE> SturmSequence<DATA_TYPE>::solveLocalMinium(const DATA_TYPE lowerBound,
                                                                  const DATA_TYPE upperBound) const
{
    std::vector<SturmInterval> intervalStorage;
    intervalStorage.reserve(this->_maxDivisions);

    int lowerBoundSign = this->countSturmSignChanges(lowerBound);
    int upperBoundSign = this->countSturmSignChanges(upperBound);
    const int totalRoots = lowerBoundSign - upperBoundSign;
    int id = 0;
    SturmInterval tmpInt(lowerBound, upperBound, lowerBoundSign, upperBoundSign, id, totalRoots);

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

    std::vector<DATA_TYPE> realRoots;
    realRoots.reserve(foundRoots);

    for (size_t i = 0; i < foundRoots; ++i) {
        DATA_TYPE root = this->solveBisectionLocalMinium(rootIntervals[i].lowerBound, rootIntervals[i].upperBound);
        if (!std::isnan(root)) {
            realRoots.emplace_back(root);
        }
    }

    return realRoots;
}

template <typename DATA_TYPE>
DATA_TYPE SturmSequence<DATA_TYPE>::solveBisectionLocalMinium(const DATA_TYPE lowerBound,
                                                              const DATA_TYPE upperBound) const
{
    DATA_TYPE lowerVal = this->_polynomial(lowerBound);
    DATA_TYPE upperVal = this->_polynomial(upperBound);

    if (!(sgn(lowerVal) ^ sgn(upperVal))) {
        return NAN;
    }

    // if (lowerVal > 0) {
    //     return boost::none;
    // }

    // if (upperVal < 0) {
    //     return boost::none;
    // }

    const int maxIterations = this->_maxDivisions;
    DATA_TYPE bisectionMin = lowerBound;
    DATA_TYPE bisectionMax = upperBound;

    DATA_TYPE mid;
    int i = 0;
    while ((bisectionMax - bisectionMin) >= this->_tolerance && i < maxIterations) {
        mid = (bisectionMax + bisectionMin) / 2.f;
        DATA_TYPE midVal = this->_polynomial(mid);

        if (fabs(midVal) <= this->_tolerance) {
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
std::vector<PowerBasisPolynomial1D<DATA_TYPE>> SturmSequence<DATA_TYPE>::generateSturmSequences() const
{
    assert(this->_polynomial.coeffs().size() > 0);

    std::vector<PowerBasisPolynomial1D<DATA_TYPE>> result;

    result.emplace_back(this->_polynomial.coeffs(), true);

    if (this->_polynomial.coeffs().size() == 1) {
        return result;
    }

    result.emplace_back(this->_polynomial.derivative().coeffs(), true);

    PowerBasisPolynomial1D<DATA_TYPE> remainderPoly =
        (std::prev(std::prev(result.end()))->divide(result.back())).second;

    while (!remainderPoly.isZero()) {
        result.emplace_back(remainderPoly.multiply(-1).coeffs(), true);
        remainderPoly = (std::prev(std::prev(result.end()))->divide(result.back())).second;
    }

    return result;
}

}  // namespace maths
}  // namespace robotics
