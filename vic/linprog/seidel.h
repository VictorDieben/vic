#pragma once

#include <cstddef>
#include <array>
#include <vector>
#include <random>

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
	//UNBOUNDED,
	INFEASIBLE,
	ERROR
};

template <typename T, std::size_t dims>
struct SeidelResult
{
	constexpr SeidelResult() = default;
	constexpr SeidelResult(ESeidelState state)
		: mState(state) {}
	constexpr SeidelResult(ESeidelState state, const std::array<T, dims>& u)
		: mState(state)
		, mU(u) {}
	constexpr SeidelResult(ESeidelState state, const std::array<T, dims>& u, const std::array<T, dims>& w)
		: mState(state)
		, mU(u)
		, mW(w) {}
	ESeidelState mState{ ESeidelState::OK };
	std::array<T, dims> mU{};
	std::array<T, dims> mW{};
};

template <typename T, std::size_t dims>
struct Constraint
{
	const T& F() const { return mData[dims]; }
	const T& G() const { return mData[dims + 1]; }
	std::array<T, dims + 2> mData{};
};


template <typename T, std::size_t dims>
class Seidel
{
public:
	using ConstraintType = Constraint<T, dims>;

	void AddConstraint(const ConstraintType& constraint)
	{
		mConstraints.push_back(constraint);
	}

	SeidelResult<T, dims> Calculate(const std::array<T, dims>& objective)
	{
		return {};
	}

	void Reset()
	{
		mConstraints.clear();
	}

private:
	std::vector<ConstraintType> mConstraints;

	std::mt19937 mGenerator{ 1234 };

	Seidel<T, dims - 1> mSubSeidel{};
};

// returns (p + lamda*q) < (r + lambda*s); for sufficient lambda
template <typename T>
constexpr bool MinL(const T p, const T q, const T r, const T s)
{
	return (q == s) ? (p < q) : (q < s);
}

// returns (p + lamda*q) > (r + lambda*s); for sufficient lambda
template <typename T>
constexpr bool MaxL(const T p, const T q, const T r, const T s)
{
	return (q == s) ? (p > q) : (q > s);
}

template <typename T>
class Seidel<T, 1>
{
public:
	using ConstraintType = Constraint<T, 1>;

	void AddConstraint(const ConstraintType& constraint)
	{
		const auto& a1 = constraint.mData[0];
		const auto& a2 = constraint.F();
		const auto& a3 = constraint.G();
		const T a2Overa1 = a2 / a1;
		const T a3Overa1 = a3 / a1;
		if (MinL(a2Overa1, a3Overa1, mH1, mH2))
		{
			mH1 = a2Overa1;
			mH2 = a3Overa1;
		}
		if (MaxL(a2Overa1, a3Overa1, mL1, mL2))
		{
			mL1 = a2Overa1;
			mL2 = a3Overa1;
		}
		if (MinL(a2, a3, mZ1, mZ2))
		{
			mZ1 = a2;
			mZ2 = a3;
		}
	}

	SeidelResult<T, 1> Calculate(const std::array<T, 1>& objective)
	{
		if (MinL(mZ1, mZ2, 0, 0) || MinL(mH1, mH2, mL1, mL2))
			return SeidelResult<T, 1>(ESeidelState::INFEASIBLE);

		if (objective.at(0) > 0.)
			return SeidelResult<T, 1>(ESeidelState::OK, { mH1 }, { mH2 });

		if (objective.at(0) < 0.)
			return SeidelResult<T, 1>(ESeidelState::OK, { mL1 }, { mL2 });

		return SeidelResult<T, 1>(ESeidelState::OK, { (mH1 + mL1) / 2. }, { 0. });
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
}

}