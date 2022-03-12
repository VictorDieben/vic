#pragma once

#include "traits.h"
#include "linalg.h"

namespace vic
{
namespace linalg
{

template <typename TMatrix>
	requires ConceptMatrix<TMatrix>
constexpr Base<typename TMatrix::DataType> Determinant(const TMatrix& matrix)
{
	if constexpr (std::is_same_v<TMatrix, Identity<TMatrix::DataType, TMatrix::Rows>>)
	{
		return Base<typename TMatrix::DataType>{1};
	}
	else if constexpr (std::is_same_v<TMatrix, Diagonal<TMatrix::DataType, TMatrix::Rows, TMatrix::Columns>> &&
		(TMatrix::Rows == TMatrix::Columns))
	{
		return DeterminantDiagonal(matrix);
	}
	else
	{
		return DeterminantGeneral(matrix);
	}
}

template <typename TMatrix>
	requires ConceptMatrix<TMatrix>
constexpr Base<typename TMatrix::DataType> DeterminantDiagonal(const TMatrix& matrix)
{
	Base<typename TMatrix::DataType> val{ 1 };
	for (std::size_t i = 0; i < TMatrix::GetRows(); ++i)
		val = val * matrix.Get(i, i);
	return val;
}


template <typename TMatrix>
	requires ConceptSquareMatrix<TMatrix>
constexpr Base<typename TMatrix::DataType> DeterminantGeneral(const TMatrix& matrix)
{
	return std::decay_t<typename TMatrix::DataType>{1};
}


}
}