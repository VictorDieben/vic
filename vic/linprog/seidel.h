#pragma once

#include <array>
#include <cstddef>
#include <numeric>
#include <random>
#include <ranges>
#include <vector>

namespace vic
{
namespace linprog
{

// seidel paper:
// https://people.eecs.berkeley.edu/~jrs/meshpapers/SeidelLP.pdf
// 'Small-dimensional linear programming and convex hulls made easey'

enum class ESeidelState
{
    OK,
    INFEASIBLE,
    ERROR
};

template <typename T, std::size_t dims>
struct SeidelResult
{
    constexpr SeidelResult() = default;
    constexpr SeidelResult(ESeidelState state)
        : mState(state)
    { }
    constexpr SeidelResult(ESeidelState state, const std::array<T, dims>& u)
        : mState(state)
        , mU(u)
    { }
    constexpr SeidelResult(ESeidelState state, const std::array<T, dims>& u, const std::array<T, dims>& w)
        : mState(state)
        , mU(u)
        , mW(w)
    { }
    ESeidelState mState{ESeidelState::OK};
    std::array<T, dims> mU{};
    std::array<T, dims> mW{};
};

template <typename T, std::size_t size1, std::size_t size2>
std::array<T, size1 - 1> ProjectArray(const std::array<T, size1>& projection, //
                                      const std::array<T, size2>& basis,
                                      const std::size_t dim)
{
    static_assert(size1 <= size2);
    assert(dim < size1);

    std::array<T, size1 - 1> res{};
    const T f = projection.at(dim) / basis.at(dim);
    std::size_t j = 0;
    for(std::size_t i = 0; i < size1; ++i)
    {
        if(i == dim)
            continue;
        res[j] = projection.at(i) - (f * basis.at(i));
        j++;
    }

    return res;
}

// returns (p + lamda*q) < (r + lambda*s); for sufficient lambda
template <typename T>
constexpr bool MinL(const T p, const T q, const T r, const T s)
{
    return (q == s) ? (p < r) : (q < s);
}

template <typename T>
constexpr bool MinEqualL(const T p, const T q, const T r, const T s)
{
    return (q == s) ? (p <= r) : (q < s);
}

// returns (p + lamda*q) > (r + lambda*s); for sufficient lambda
template <typename T>
constexpr bool MaxL(const T p, const T q, const T r, const T s)
{
    return (q == s) ? (p > r) : (q > s);
}

template <typename T, std::size_t dims>
struct Constraint
{
    Constraint() = default;
    Constraint(const std::array<T, dims>& a, const T f)
        : Constraint(a, f, 0.)
    { }
    Constraint(const std::array<T, dims>& a, const T f, const T g)
    {
        for(std::size_t i = 0; i < dims; ++i)
            mData[i] = a[i];
        mData[dims] = f;
        mData[dims + 1] = g;
    }
    const T& F() const { return mData[dims]; }
    const T& G() const { return mData[dims + 1]; }
    std::array<T, dims + 2> mData{};

    const auto A() const { return std::ranges::subrange{mData.begin(), mData.end() - 2}; }
};

template <typename T, std::size_t dims>
class Seidel
{
public:
    using ConstraintType = Constraint<T, dims>;
    using ResultType = SeidelResult<T, dims>;

    void AddConstraint(const ConstraintType& constraint) { mConstraints.push_back(constraint); }

    SeidelResult<T, dims> Calculate(const std::array<T, dims>& objective)
    {
        // TODO(vicdie): shuffel constraints
        std::shuffle(mConstraints.begin(), mConstraints.end(), mGenerator);

        // TODO(vicdie): select one constraint, check if it still satisfied
        // - if it is, go to next constraint
        // - if it is not, calculate new solution in lower dimension, project, continue
        ResultType result(ESeidelState::OK, {}, objective);

        // TODO(vicdie): check norm of objective

        for(std::size_t i = 0; i < mConstraints.size(); ++i)
        {
            //
            const auto& a = mConstraints.at(i);

            if(ConstraintIsSatisfied(a, result.mU, result.mW, mEpsilon))
                continue;

            // select dimension to project onto
            const std::size_t k = SelectDimension(a);
            const T ak = a.mData.at(k);
            if(std::fabs(ak) < mEpsilon)
                return ResultType(ESeidelState::INFEASIBLE);

            mSubSeidel.Reset();

            // project all constraints so far onto d-1 subproblem
            for(std::size_t j = 0; j < i; ++j)
            {
                const auto projectedConstraint = ProjectConstraint(mConstraints.at(j), a, k);
                mSubSeidel.AddConstraint(projectedConstraint);
            }

            // solve d-1 subproblem
            const auto subObjective = ProjectObjective(objective, a, k);
            const SeidelResult<T, dims - 1> sub = mSubSeidel.Calculate(subObjective);
            if(sub.mState != ESeidelState::OK)
                return ResultType(sub.mState);

            // lift d-1-dimensional solution back to d dimensional space
            result = LiftSolution(sub, a, k);
        }
        return result;
    }

    ResultType LiftSolution(const SeidelResult<T, dims - 1>& subSolution, //
                            const ConstraintType& constraint,
                            const std::size_t dim) const
    {
        ResultType res(subSolution.mState);
        std::size_t j = 0;
        for(std::size_t i = 0; i < dims; ++i)
        {
            if(i == dim)
                continue;
            res.mU[i] = subSolution.mU.at(j);
            res.mW[i] = subSolution.mW.at(j);
            j++;
        }
        const T udot = std::inner_product(res.mU.begin(), res.mU.end(), constraint.mData.begin(), 0.);
        const T wdot = std::inner_product(res.mW.begin(), res.mW.end(), constraint.mData.begin(), 0.);
        res.mU[dim] = (constraint.F() - udot) / constraint.mData.at(dim);
        res.mW[dim] = (constraint.G() - wdot) / constraint.mData.at(dim);
        return res;
    }

    bool ConstraintIsSatisfied(const ConstraintType& constraint, //
                               const std::array<T, dims>& u,
                               const std::array<T, dims>& w,
                               const T& eps) const
    {
        const T udot = std::inner_product(u.begin(), u.end(), constraint.mData.begin(), 0.);
        const T wdot = std::inner_product(w.begin(), w.end(), constraint.mData.begin(), 0.);
        return MinEqualL(udot, wdot, constraint.F(), constraint.G());
    }

    std::array<T, dims - 1> ProjectObjective(const std::array<T, dims>& objective, //
                                             const ConstraintType& constraint,
                                             const std::size_t dim) const
    {
        return ProjectArray(objective, constraint.mData, dim);
    }

    Constraint<T, dims - 1> ProjectConstraint(const ConstraintType& constraint, //
                                              const ConstraintType& onto,
                                              const std::size_t dim) const
    {
        Constraint<T, dims - 1> res{};
        res.mData = ProjectArray(constraint.mData, onto.mData, dim);
        return res;
    }

    std::size_t SelectDimension(const ConstraintType& constraint) const
    {
        const auto predicate = [](const T& lhs, const T& rhs) { return std::fabs(lhs) < std::fabs(rhs); };
        auto k = std::max_element(constraint.mData.begin(), constraint.mData.end() - 2, predicate);
        return (k - constraint.mData.begin());
    }

    void Reset() { mConstraints.clear(); }

private:
    std::vector<ConstraintType> mConstraints;

    std::mt19937 mGenerator{1234};

    Seidel<T, dims - 1> mSubSeidel{};

    T mEpsilon{1e-14};
};

template <typename T>
class Seidel<T, 1>
{
public:
    Seidel() { Reset(); }
    using ConstraintType = Constraint<T, 1>;

    void AddConstraint(const ConstraintType& constraint)
    {
        const auto& a1 = constraint.mData[0];
        const auto& a2 = constraint.F();
        const auto& a3 = constraint.G();
        const T a2Overa1 = a2 / a1;
        const T a3Overa1 = a3 / a1;
        if(a1 > 0.)
        {
            if(MinL(a2Overa1, a3Overa1, mH1, mH2))
            {
                mH1 = a2Overa1;
                mH2 = a3Overa1;
            }
        }
        else if(a1 < 0.)
        {
            if(MaxL(a2Overa1, a3Overa1, mL1, mL2))
            {
                mL1 = a2Overa1;
                mL2 = a3Overa1;
            }
        }
        else if(a1 == 0. && MinL(a2, a3, mZ1, mZ2))
        {
            mZ1 = a2;
            mZ2 = a3;
        }
    }

    SeidelResult<T, 1> Calculate(const std::array<T, 1>& objective)
    {
        if(MinL<T>(mZ1, mZ2, 0, 0) || MinL<T>(mH1, mH2, mL1, mL2))
            return SeidelResult<T, 1>(ESeidelState::INFEASIBLE);

        if(objective.at(0) > 0.)
            return SeidelResult<T, 1>(ESeidelState::OK, {mH1}, {mH2});

        if(objective.at(0) < 0.)
            return SeidelResult<T, 1>(ESeidelState::OK, {mL1}, {mL2});

        return SeidelResult<T, 1>(ESeidelState::OK, {(mH1 + mL1) / 2.}, {0.});
    }

    void Reset()
    {
        mH1 = 0.;
        mH2 = 1.;
        mL1 = 0.;
        mL2 = -1.;
        mZ1 = 0.;
        mZ2 = 0.;
    }

private:
    T mL1{}, mL2{}, mZ1{}, mZ2{}, mH1{}, mH2{};
};
} // namespace linprog

} // namespace vic