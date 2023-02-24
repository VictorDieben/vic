#pragma once

// This file implements the variable size matrices.
// if memory or computational resources are limited,
// do not include this file. It is not needed for fixed size computations.
#include "vic/linalg/matrices.h"
#include "vic/linalg/traits.h"

#include "vic/memory/flat_map.h"

#include <vector>

namespace vic
{
namespace linalg
{

// standard constant size matrix (row mayor)
template <typename T>
class MatrixDynamic
{
public:
    using DataType = T;

    constexpr static auto Size = ESizeType::Dynamic;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;

    MatrixDynamic() = default;
    explicit MatrixDynamic(const std::size_t rows, const std::size_t cols)
        : mRows(rows)
        , mColumns(cols)
    {
        Initialize();
    }
    explicit MatrixDynamic(const std::size_t rows, const std::size_t cols, const std::vector<T>& data)
        : mData(data)
        , mRows(rows)
        , mColumns(cols)
    {
        assert(rows * cols == data.size()); // make sure data is correct length
    }

    template <typename TMat>
    requires ConceptMatrix<TMat>
    constexpr explicit MatrixDynamic(const TMat& matrix)
    {
        mRows = matrix.GetRows();
        mColumns = matrix.GetColumns();
        Initialize();
        for(std::size_t i = 0; i < mRows; ++i)
            for(std::size_t j = 0; j < mColumns; ++j)
                At(i, j) = matrix.Get(i, j);
    }

    std::size_t GetRows() const { return mRows; }
    std::size_t GetColumns() const { return mColumns; }

    constexpr T Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return mData[RowColToIndex(i, j, GetColumns())];
    }

    constexpr T& At(const std::size_t i, const std::size_t j)
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return mData[RowColToIndex(i, j, GetColumns())];
    }

private:
    // TODO: maybe std::vector is not the right fit,
    // we don't want to resize after initializing
    std::vector<T> mData{};
    std::size_t mRows{0};
    std::size_t mColumns{0};

    void Initialize() { mData.resize(mRows * mColumns); }
};

// TODO: make ColMajor
template <typename T, std::size_t rows>
class MatrixRowConst
{
public:
    using DataType = T;
    constexpr static auto Size = ESizeType::RowConstant;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;

    MatrixRowConst(const std::size_t columns)
        : mColumns(columns)
    {
        Initialize();
    }
    MatrixRowConst(const std::size_t validate_rows, const std::size_t columns)
        : mColumns(columns)
    {
        assert(validate_rows == rows);
        Initialize();
    }
    constexpr static std::size_t GetRows() { return rows; }
    std::size_t GetColumns() const { return mColumns; }

    T Get(const std::size_t i, const std::size_t j) const { return mData.at(RowColToIndex(i, j, GetColumns())); }

    T& At(const std::size_t i, const std::size_t j)
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return mData[RowColToIndex(i, j, GetColumns())];
    }

private:
    std::size_t mColumns{};
    std::vector<T> mData{};

    void Initialize() { mData.resize(rows * mColumns); }
};

template <typename T, std::size_t columns>
class MatrixColConst
{
public:
    using DataType = T;
    constexpr static auto Size = ESizeType::ColConstant;
    constexpr static auto Ordering = EOrdering::ColumnMayor;
    constexpr static auto Distribution = EDistribution::Full;

    MatrixColConst(const std::size_t rows)
        : mRows(rows)
    {
        Initialize();
    }
    MatrixColConst(const std::size_t rows, const std::size_t validate_columns)
        : mRows(rows)
    {
        assert(columns == validate_columns);
        Initialize();
    }
    std::size_t GetRows() const { return mRows; }
    constexpr static std::size_t GetColumns() { return columns; }

    T Get(const std::size_t i, const std::size_t j) const { return mData.at(RowColToIndex(i, j, GetColumns())); }

    T& At(const std::size_t i, const std::size_t j)
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return mData[RowColToIndex(i, j, GetColumns())];
    }

private:
    std::size_t mRows{};
    std::vector<T> mData{};

    void Initialize() { mData.resize(mRows * columns); }
};

template <typename T>
class VectorDynamic
{
public:
    using DataType = T;
    constexpr static auto Size = ESizeType::Dynamic;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;

    VectorDynamic() = default;
    VectorDynamic(std::size_t size) { mData.resize(size); }
    VectorDynamic(const std::vector<T>& data)
        : mData(data)
    { }

    template <typename TMatrix>
    explicit VectorDynamic(const TMatrix& other)
    {
        assert(other.GetColumns() == 1);
        mData.resize(other.GetRows());
        for(std::size_t i = 0; i < other.GetRows(); ++i)
        {
            mData[i] = other.Get(i, 0); // todo: copy underlying std::vector directly
        }
    }

    // todo: transpose?
    std::size_t GetRows() const { return mData.size(); }
    std::size_t GetColumns() const { return 1; }

    T Get(const std::size_t i, const std::size_t j) const
    {
        assert(j == 0);
        return mData[i];
    }

    T& At(const std::size_t i, const std::size_t j)
    {
        assert(j == 0);
        return mData[i];
    }

private:
    std::vector<T> mData;
};

template <typename T>
class DiagonalDynamic
{
public:
    using DataType = T;
    constexpr static auto Size = ESizeType::Dynamic;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Diagonal;

    DiagonalDynamic() = default;
    explicit DiagonalDynamic(const std::size_t rows, const std::size_t cols)
        : mRows(rows)
        , mColumns(cols)
    {
        mData.resize(std::min(mRows, mColumns));
    }
    explicit DiagonalDynamic(const std::size_t rows, const std::size_t cols, const std::vector<T>& data)
        : mData(data)
        , mRows(rows)
        , mColumns(cols)
    {
        assert(rows * cols == data.size()); // make sure data is correct length
    }

    template <typename TMatrix>
    requires ConceptMatrix<TMatrix>
    explicit DiagonalDynamic(const TMatrix& other)
        : mRows(other.GetRows())
        , mColumns(other.GetColumns())
    {
        const auto n = Min(other.GetRows(), other.GetColumns());
        mData.resize(n);
        for(std::size_t i = 0; i < n; ++i)
            mData[i] = other.Get(i, i);
    }

    // todo: transpose?
    std::size_t GetRows() const { return mRows; }
    std::size_t GetColumns() const { return mColumns; }

    T Get(const std::size_t i, const std::size_t j) const
    {
        if(i == j)
            return mData[i];
        return 0.;
    }

    T& At(const std::size_t i, const std::size_t j)
    {
        assert(i == j);
        return mData[i];
    }

private:
    std::vector<T> mData;
    std::size_t mRows{0};
    std::size_t mColumns{0};
};

} // namespace linalg
} // namespace vic