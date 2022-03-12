#pragma once

// This file implements the variable size matrices
#include "vic/linalg/matrices.h"
#include "vic/linalg/traits.h"

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
    std::vector<T> mData{};
    std::size_t mRows{0};
    std::size_t mColumns{0};

    void Initialize()
    {
        //
        mData.reserve(mRows * mColumns);
    }
};

} // namespace linalg
} // namespace vic