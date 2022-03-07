#pragma once

#include "traits.h"

namespace vic
{
namespace linalg
{


template <typename TMatrix>
constexpr auto AddConstant(const TMatrix& matrix, const typename TMatrix::DataType& value)
{
	Matrix<TMatrix::DataType, TMatrix::Rows, TMatrix::Columns> result;
	for (std::size_t i = 0; i < TMatrix::Rows; ++i)
		for (std::size_t j = 0; j < TMatrix::Columns; ++j)
			result.At(i, j) = matrix.Get(i, j) + value;
	return result;
}

template <typename TMat1, typename TMat2, std::enable_if_t<((IsMatrix<TMat1>::value&& IsMatrix<TMat2>::value)), bool> = true>
constexpr auto AddGeneral(const TMat1& mat1, const TMat2& mat2)
{
	Matrix<double, TMat1::Rows, TMat1::Columns> result;
	for (std::size_t i = 0; i < TMat1::Rows; ++i)
		for (std::size_t j = 0; j < TMat1::Columns; ++j)
			result.At(i, j) = mat1.Get(i, j) + mat2.Get(i, j);
	return result;
}

template <typename TMat1, typename TMat2>
constexpr auto Add(const TMat1& mat1, const TMat2& mat2)
{
	// add constant to matrix
	constexpr bool isNumber1 = IsFloatOrIntegral<std::decay_t<TMat1>>::value;
	constexpr bool isNumber2 = IsFloatOrIntegral<std::decay_t<TMat2>>::value;
	if constexpr (isNumber1 && isNumber2)
		return mat1 + mat2;

	else if constexpr (isNumber1 && !isNumber2)
		return AddConstant(mat2, mat1);

	else if constexpr (!isNumber1 && isNumber2)
		return AddConstant(mat1, mat2);

	else
		return AddGeneral(mat1, mat2);
}



}
}