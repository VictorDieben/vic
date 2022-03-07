#pragma once


#include "vic/linalg/linalg.h"


namespace vic
{
namespace linalg
{

// trace is sum of all diagonal elements
template <typename TMat>
constexpr auto Trace(const TMat& matrix)
{
	static_assert(IsSquare<TMat>::value);
	TMat::DataType val{ 0 };
	for (std::size_t i = 0; i < TMat::Rows; ++i)
		val += matrix.Get(i, i);
	return val;
}

// 2d cross product
template <typename T>
constexpr auto Cross(const Vector2<T>& vec1, const Vector2<T>& vec2)
{
	return (vec1.Get(0, 0) * vec2.Get(1, 0)) - (vec1.Get(1, 0) * vec2.Get(0, 0));
}

// 3d cross product
template <typename T>
constexpr auto Cross(const Vector3<T>& vec1, const Vector3<T>& vec2)
{
	const double ax = vec1.Get(0, 0), ay = vec1.Get(1, 0), az = vec1.Get(1, 0);
	const double bx = vec2.Get(0, 0), by = vec2.Get(1, 0), bz = vec2.Get(1, 0);
	return Vector3<double>({ (ay * bz) - (az * by),			//
							 (az * bx) - (ax * bz),			//
							 (ax * by) - (ay * bx) });
}

// dot product between two vectors
template <typename TMat1, typename TMat2>
constexpr auto Dot(const TMat1& mat1, const TMat2& mat2)
{
	static_assert(TMat1::Columns == 1 && TMat2::Columns == 1 && TMat1::Rows == TMat2::Rows);
	using TRet = typename  decltype(typename TMat1::DatType{} *typename TMat2::DataType{});
	TRet val{ 0 };
	for (std::size_t i = 0; i < TMat1::Rows; ++i)
		val += (mat1.Get(i, 0) * mat2.Get(i, 0));
	return val;
}

}
}