#pragma once

#include "assert.h"
#include "vic/linalg/traits.h"
#include "vic/utils.h"

#include <array>
#include <cstddef>

namespace vic
{
namespace linalg
{

// get row mayor index of i,j pair
template <std::size_t columns>
constexpr std::size_t RowColToIndex(const std::size_t i, const std::size_t j)
{
    return (i * columns) + j;
}

// get row mayor index of i,j pair
constexpr std::size_t RowColToIndex(const std::size_t i, const std::size_t j, const std::size_t columns)
{
    return (i * columns) + j; //
}

// standard constant size matrix (row mayor)
template <typename T, std::size_t rows, std::size_t columns>
class Matrix
{
public:
    using DataType = T;
    constexpr Matrix() = default;

    constexpr explicit Matrix(const T& value)
    {
        for(auto& item : mData)
            item = value;
    }

    constexpr explicit Matrix(const std::array<T, rows * columns>& data)
        : mData(data)
    { }

    template <typename TMat>
    requires ConceptMatrix<TMat>
    constexpr explicit Matrix(const TMat& matrix)
    {
        for(std::size_t i = 0; i < rows; ++i)
            for(std::size_t j = 0; j < columns; ++j)
                At(i, j) = matrix.Get(i, j);
    }

    constexpr static std::size_t GetRows() { return rows; }
    constexpr static std::size_t GetColumns() { return columns; }

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < rows) && (j < columns)));
        return mData[RowColToIndex<columns>(i, j)];
    }
    constexpr T Get(const std::size_t i) const
    {
        static_assert(rows == 1 || columns == 1);
        assert(i < mData.size());
        return mData.at(i);
    }

    constexpr T& At(const std::size_t i, const std::size_t j)
    {
        assert(((i < rows) && (j < columns)));
        return mData[RowColToIndex<columns>(i, j)];
    }

    // NOTE: not private, do with it what you want
    std::array<T, rows * columns> mData{};
};

template <typename T>
using Matrix2 = Matrix<T, 2, 2>;
template <typename T>
using Matrix3 = Matrix<T, 3, 3>;
template <typename T>
using Matrix4 = Matrix<T, 4, 4>;
template <typename T>
using Matrix5 = Matrix<T, 5, 5>;
template <typename T>
using Matrix6 = Matrix<T, 6, 6>;
template <typename T>
using Vector1 = Matrix<T, 1, 1>;
template <typename T>
using Vector2 = Matrix<T, 2, 1>;
template <typename T>
using Vector3 = Matrix<T, 3, 1>;
template <typename T>
using Vector4 = Matrix<T, 4, 1>;
template <typename T>
using Vector5 = Matrix<T, 5, 1>;
template <typename T>
using Vector6 = Matrix<T, 6, 1>;

template <typename T, std::size_t n>
using Vector = Matrix<T, n, 1>;

template <typename T, std::size_t rows, std::size_t columns>
class Zeros
{
public:
    constexpr Zeros() = default;

    constexpr static std::size_t GetRows() { return rows; }
    constexpr static std::size_t GetColumns() { return columns; }

    using DataType = T;

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return T{0};
    }
};

template <typename T, std::size_t rows, std::size_t columns>
class Ones
{
public:
    constexpr Ones() = default;

    constexpr static std::size_t GetRows() { return rows; }
    constexpr static std::size_t GetColumns() { return columns; }

    using DataType = T;

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return T{1};
    }
};

template <typename T, std::size_t size>
class Identity
{
public:
    constexpr Identity() = default;

    constexpr static std::size_t GetRows() { return size; }
    constexpr static std::size_t GetColumns() { return size; }

    using DataType = T;

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return i == j ? T(1) : T(0);
    }
};

template <typename T, std::size_t size>
class DiagonalConstant
{
public:
    constexpr DiagonalConstant() = default;
    constexpr DiagonalConstant(const T value)
        : mValue(value)
    { }

    constexpr static std::size_t GetRows() { return size; }
    constexpr static std::size_t GetColumns() { return size; }

    using DataType = T;

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return i == j ? mValue : T{0};
    }

private:
    const T mValue{0.};
};

// diagonal matrix, only has values where i==j.
// does not have to be square
template <typename T, std::size_t rows, std::size_t columns>
class Diagonal
{
public:
    constexpr Diagonal() = default;
    constexpr explicit Diagonal(const std::array<T, Min(rows, columns)>& data)
        : mData(data)
    { }
    constexpr explicit Diagonal(const T& value)
    {
        for(std::size_t i = 0; i < Min(rows, columns); ++i)
            mData[i] = value;
    }
    template <typename TMat>
    requires ConceptMatrix<TMat>
    constexpr explicit Diagonal(const TMat& matrix)
    {
        for(std::size_t i = 0; i < Min(rows, columns); ++i)
            mData[i] = matrix.Get(i, i);
    }

    constexpr static std::size_t GetRows() { return rows; }
    constexpr static std::size_t GetColumns() { return columns; }

    using DataType = T;

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return i == j ? mData.at(i) : 0.;
    }

    constexpr T& At(const std::size_t i, const std::size_t j)
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        assert(i == j); // only diagonal can be set
        return mData[i];
    }

    // NOTE: not private, do with it what you want
    std::array<T, Min(rows, columns)> mData{};
};

template <typename TFunctor, std::size_t rows, std::size_t columns>
class LambdaMatrix
{
    TFunctor mFunctor;

public:
    constexpr LambdaMatrix(TFunctor functor)
        : mFunctor(functor)
    { }

    constexpr static std::size_t GetRows() { return rows; }
    constexpr static std::size_t GetColumns() { return columns; }

    using DataType = decltype(mFunctor(std::size_t{}, std::size_t{}));

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return mFunctor(i, j);
    }
};

} // namespace linalg
} // namespace vic