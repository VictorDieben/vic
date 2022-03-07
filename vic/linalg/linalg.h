#pragma once

#include <cstddef>
#include <array>
#include "vic/linalg/traits.h"
#include "vic/linalg/matmul.h"
#include "vic/linalg/add.h"


namespace vic
{
template <typename T>
constexpr T Min(const T& val1, const T& val2)
{
	return val1 < val2 ? val1 : val2;
}
template <typename T>
constexpr T Max(const T& val1, const T& val2)
{
	return val1 > val2 ? val1 : val2;
}

namespace linalg
{


// get row mayor index of i,j pair
template <std::size_t columns>
constexpr std::size_t RowColToIndex(const std::size_t i, const std::size_t j)
{
	return (i * columns) + j;
}

// standard constant size matrix (row mayor)
template <typename T, std::size_t rows, std::size_t columns>
class Matrix
{
public:
	constexpr Matrix() = default;
	constexpr explicit Matrix(const std::array<T, rows* columns>& data)
		: mData(data) { }

	template <typename TMat, std::enable_if_t<IsMatrix<TMat>::value, bool> = true >
	constexpr explicit Matrix(const TMat& matrix)
	{
		for (std::size_t i = 0; i < rows; ++i)
			for (std::size_t j = 0; j < columns; ++j)
				At(i, j) = matrix.Get(i, j);
	}
	constexpr static size_t Rows = rows;
	constexpr static size_t Columns = columns;
	using DataType = T;

	constexpr static std::size_t GetRows() { return rows; }
	constexpr static std::size_t GetColumns() { return columns; }

	constexpr T Get(const std::size_t i, const std::size_t j) const
	{
		assert(((i < Rows) && (j < Columns)));
		return mData[RowColToIndex<columns>(i, j)];
	}

	constexpr T& At(const std::size_t i, const std::size_t j)
	{
		assert(((i < Rows) && (j < Columns)));
		return mData[RowColToIndex<columns>(i, j)];
	}

	// NOTE: not private, do with it what you want
	std::array<T, rows* columns> mData{};
};

template <typename T> using Matrix2 = Matrix<T, 2, 2>;
template <typename T> using Matrix3 = Matrix<T, 3, 3>;
template <typename T> using Matrix4 = Matrix<T, 4, 4>;
template <typename T> using Matrix5 = Matrix<T, 5, 5>;
template <typename T> using Matrix6 = Matrix<T, 6, 6>;
template <typename T> using Vector1 = Matrix<T, 1, 1>;
template <typename T> using Vector2 = Matrix<T, 2, 1>;
template <typename T> using Vector3 = Matrix<T, 3, 1>;
template <typename T> using Vector4 = Matrix<T, 4, 1>;
template <typename T> using Vector5 = Matrix<T, 5, 1>;
template <typename T> using Vector6 = Matrix<T, 6, 1>;


template <typename T, std::size_t rows, std::size_t columns>
class Zeros
{
public:
	constexpr Zeros() = default;

	constexpr static size_t Rows = rows;
	constexpr static size_t Columns = columns;
	constexpr static std::size_t GetRows() { return Rows; }
	constexpr static std::size_t GetColumns() { return Columns; }

	using DataType = T;

	constexpr T Get(const std::size_t i, const std::size_t j) const
	{
		assert(((i < Rows) && (j < Columns)));
		return T{ 0 };
	}
};

template <typename T, std::size_t size>
class Identity
{
public:
	constexpr Identity() = default;

	constexpr static size_t Rows = size;
	constexpr static size_t Columns = size;
	constexpr static std::size_t GetRows() { return Rows; }
	constexpr static std::size_t GetColumns() { return Columns; }

	using DataType = T;

	constexpr T Get(const std::size_t i, const std::size_t j) const
	{
		assert(((i < Rows) && (j < Columns)));
		return i == j ? T(1.) : T(0.);
	}
};

// diagonal matrix, only has values where i==j.
// does not have to be square
template <typename T, std::size_t rows, std::size_t columns>
class Diagonal
{
public:
	constexpr Diagonal() = default;
	constexpr explicit Diagonal(const std::array<T, Min(rows, columns)>& data)
		: mData(data) {}
	constexpr explicit Diagonal(const T& value)
	{
		for (std::size_t i = 0; i < Min(rows, columns); ++i)
			mData[i] = value;
	}
	template <typename TMat, std::enable_if_t<IsMatrix<TMat>::value, bool> = true >
	constexpr explicit Diagonal(const TMat& matrix)
	{
		for (std::size_t i = 0; i < Min(rows, columns); ++i)
			mData[i] = matrix.Get(i, i);
	}

	constexpr static std::size_t Rows = rows;
	constexpr static std::size_t Columns = columns;
	constexpr static std::size_t GetRows() { return rows; }
	constexpr static std::size_t GetColumns() { return columns; }

	using DataType = T;

	constexpr T Get(const std::size_t i, const std::size_t j) const
	{
		assert(((i < Rows) && (j < Columns)));
		return i == j ? mData.at(i) : 0.;
	}

	constexpr T& At(const std::size_t i, const std::size_t j)
	{
		assert(((i < Rows) && (j < Columns)));
		assert(i == j);
		return mData[i];
	}

	// NOTE: not private, do with it what you want
	std::array<T, Min(rows, columns)> mData{};
};

}
}