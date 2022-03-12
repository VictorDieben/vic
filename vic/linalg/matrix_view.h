#pragma once

#include "vic/linalg/linalg.h"

namespace vic
{
namespace linalg
{

// A view is an object that changes how the passed in matrix is seen on the outside.
// A view is read only, similar to std::string_view.
// It also does not have ownership, lifetime needs to be managed externally.
// Matrices can be constructed from views.

template <typename TMatrix>
class ViewTranspose
{
public:
    ViewTranspose(const TMatrix& matrix)
        : mMatrix(matrix) { }

    using DataType = typename TMatrix::DataType;
    constexpr static size_t Rows = TMatrix::Columns;
    constexpr static size_t Columns = TMatrix::Rows;
    constexpr static std::size_t GetRows() { return Rows; }
    constexpr static std::size_t GetColumns() { return Columns; }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < Rows) && (j < Columns)));
        return mMatrix.Get(j, i);
    }

private:
    const TMatrix& mMatrix;
};

template <typename TMat1, typename TMat2>
class ViewRowStack
{
public:
    static_assert(TMat1::Columns == TMat2::Columns);
    static_assert(std::is_same_v<TMat1::DataType, TMat2::DataType>);

    ViewRowStack(const TMat1& mat1, const TMat2& mat2)
        : mMatrix1(mat1)
        , mMatrix2(mat2) {}

    using DataType = typename TMat1::DataType;
    constexpr static size_t Rows = TMat1::Rows + TMat2::Rows;
    constexpr static size_t Columns = TMat1::Columns;
    constexpr static std::size_t GetRows() { return Rows; }
    constexpr static std::size_t GetColumns() { return Columns; }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < Rows) && (j < Columns)));
        if (i < TMat1::Rows)
            return mMatrix1.Get(i, j);
        else
            return mMatrix2.Get(i - TMat1::Rows, j);
    }

private:
    const TMat1& mMatrix1;
    const TMat2& mMatrix2;
};

template <typename TMat1, typename TMat2>
class ViewColumnStack
{
public:
    static_assert(TMat1::Rows == TMat2::Rows);
    static_assert(std::is_same_v<TMat1::DataType, TMat2::DataType>);

    ViewColumnStack(const TMat1& mat1, const TMat2& mat2)
        : mMatrix1(mat1)
        , mMatrix2(mat2) {}

    using DataType = typename TMat1::DataType;
    constexpr static size_t Rows = TMat1::Rows;
    constexpr static size_t Columns = TMat1::Columns + TMat2::Columns;
    constexpr static std::size_t GetRows() { return Rows; }
    constexpr static std::size_t GetColumns() { return Columns; }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < Rows) && (j < Columns)));
        if (j < TMat1::Columns)
            return mMatrix1.Get(i, j);
        else
            return mMatrix2.Get(i, j - TMat1::Columns);
    }

private:
    const TMat1& mMatrix1;
    const TMat2& mMatrix2;
};

// put several matrices along the diagonal
// This object can be nested, e.g. diag(diag(a, b), diag(c, d))
template <typename TMat1, typename TMat2>
    requires ConceptSquareMatrix<TMat1> && ConceptSquareMatrix<TMat2> && HasSameType<TMat1, TMat2>::value
class ViewBlockDiagonal
{
public:

    ViewBlockDiagonal(const TMat1& mat1, const TMat2& mat2)
        : mMatrix1(mat1)
        , mMatrix2(mat2) {}

    using DataType = typename TMat1::DataType;
    constexpr static size_t Rows = TMat1::Rows + TMat2::Rows;
    constexpr static size_t Columns = TMat1::Columns + TMat2::Columns;
    constexpr static std::size_t GetRows() { return Rows; }
    constexpr static std::size_t GetColumns() { return Columns; }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < Rows) && (j < Columns)));
        constexpr const static std::size_t n = TMat1::Columns;
        if (i < n)
            if (j < n)
                return mMatrix1.Get(i, j);
            else
                return DataType{ 0 };
        else
            if (j < n)
                return DataType{ 0 };
            else
                return mMatrix2.Get(i - n, j - n);
    }
private:
    const TMat1& mMatrix1;
    const TMat2& mMatrix2;
};

// TODO(vicdie): blockdiagonal<T..>

// TODO(vicdie): submatrix
template <typename TMat, std::size_t rowStart, std::size_t rows, std::size_t colStart, std::size_t cols>
    requires ConceptMatrix<TMat> // && (rowStart + rows <= TMat::Rows()) && (colStart + cols <= TMat::GetColumns())
class ViewSubMatrix
{
public:
    ViewSubMatrix(const TMat& mat)
        : mMatrix(mat) {}

    using DataType = typename TMat::DataType;
    constexpr static size_t Rows = rows;
    constexpr static size_t Columns = cols;
    constexpr static std::size_t GetRows() { return Rows; }
    constexpr static std::size_t GetColumns() { return Columns; }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        return mMatrix.Get(i+ rowStart, j + colStart);
    }
private:
    const TMat& mMatrix;
};
}
}