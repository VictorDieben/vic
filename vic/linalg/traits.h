#pragma once

#include <type_traits>

// This file specifies the requirements for what a matrix/vector constitutes

namespace vic
{
namespace linalg
{


template <typename T>
using Base = std::remove_const_t<std::decay_t<T>>;

template <typename TMat>
struct IsSquare
{
	constexpr static bool value = (TMat::Rows == TMat::Columns);
};

template <typename TMat>
struct IsDiagonal
{
	constexpr static bool value = IsSquare<TMat>::value; // todo: make this a property of matrix types
};

template <typename T1, typename T2>
struct HasSameType
{
	constexpr static bool value = std::is_same_v<T1::DataType, T2::DataType>;
};

//template <typename T1, typename T2>
//struct HasSameShape
//{
//	constexpr static bool value = false; // default value
//};
//template <typename T1, typename T2,
//	typename = std::enable_if_t< (IsMatrix<T1>::value&& IsMatrix<T2>::value)>::value >
//	struct HasSameShape
//{
//	constexpr static bool value = ((T1::Rows == T2::Rows) && (T1::Columns == T2::Columns));
//};


template <typename T>
struct HasGet
{
	constexpr static bool value = true; // todo
};

template <typename T>
struct IsFloatOrIntegral
{
	constexpr static bool value = (std::is_integral_v<T> || std::is_floating_point_v<T>);
};

// check if T has:
// - a data type,
// - a number of rows,
// - a number of columns
// - a Get(i,j) member
template <typename T>
struct IsMatrix
{
	constexpr static bool value = !IsFloatOrIntegral<std::decay_t<T>>::value;
};


template <typename TMatrix>
constexpr bool IsPositiveDefinite(const TMatrix& matrix)
{
	return true; // TODO(vicdie): implement
}


template <typename TMatrix>
constexpr bool IsTotallyPositive(const TMatrix& matrix)
{
	if constexpr (!IsSquare(matrix))
	{
		return false;
	}
	else
	{
		return true; // TODO(vicdie): implement
	}
}

}
}