#pragma once

// This file implements the matrix multiplication for all different types of matrices


namespace vic
{
namespace linalg
{

// selector for the correct type of matrix multiplication
template <typename TMat1, typename TMat2>
constexpr auto Matmul(const TMat1& mat1, const TMat2& mat2)
{
	constexpr bool isFloat1 = IsFloatOrIntegral<std::decay_t<TMat1>>::value;
	constexpr bool isFloat2 = IsFloatOrIntegral<std::decay_t<TMat2>>::value;

	if constexpr (isFloat1 && isFloat2)
		return mat1 * mat2;

	else if constexpr (isFloat1 && !isFloat2)
		return MatmulScalar(mat2, mat1);

	else if constexpr (!isFloat1 && isFloat2)
		return MatmulScalar(mat1, mat2);

	else
	{
		// if we are not multiplying with values, check shape
		static_assert(TMat1::GetColumns() == TMat2::GetRows());
		auto res = MatmulGeneral(mat1, mat2);
		return res;
	}
}


// multiplication of two equal matrix types
template <typename TMat, typename TFloat, class TRet = decltype(typename TMat::DataType()* TFloat())>
constexpr auto MatmulScalar(const TMat& mat, const TFloat& scalar)
{
	using T = typename TMat::DataType;
	constexpr auto rows = TMat::Rows;
	constexpr auto cols = TMat::Columns;

	if constexpr (std::is_same_v<TMat, Zeros<T, rows, cols>>)
	{
		return Zeros<T, rows, cols>{};
	}
	else  if constexpr (std::is_same_v<TMat, Identity<T, rows>>)
	{
		return Diagonal<T, rows, cols>{scalar};
	}
	else if constexpr (std::is_same_v<TMat, Diagonal<T, rows, cols>>)
	{
		Diagonal<T, rows, cols> result{};
		for (std::size_t i = 0; i < Min(rows, cols); ++i)
			result.At(i, i) = mat.Get(i, i) * scalar;
		return result;
	}
	else
	{
		Matrix<TRet, rows, cols> result{};
		for (std::size_t i = 0; i < rows; ++i)
			for (std::size_t j = 0; j < cols; ++j)
				result.At(i, j) = mat.Get(i, j) * scalar;
		return result;
	}
}


// multiplication of any 2 matrices for which no useful overload exists
template <typename TMat1, typename TMat2, class TRet = decltype(typename TMat1::DataType()* typename TMat2::DataType())>
constexpr auto MatmulGeneral(const TMat1& mat1, const TMat2& mat2)
{
	static_assert(TMat1::GetColumns() == TMat2::GetRows());
	Matrix<TRet, TMat1::GetRows(), TMat2::GetColumns()> result{};
	for (std::size_t i = 0; i < TMat1::GetRows(); ++i)
		for (std::size_t j = 0; j < TMat2::GetColumns(); ++j)
			for (std::size_t k = 0; k < TMat1::GetColumns(); ++k)
				result.At(i, j) += (mat1.Get(i, k) * mat2.Get(k, j));
	return result;
}

}
}