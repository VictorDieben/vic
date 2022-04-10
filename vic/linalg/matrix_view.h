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
requires ConceptMatrix<TMatrix>
class ViewTranspose
{
public:
    ViewTranspose(const TMatrix& matrix)
        : mMatrix(matrix)
    { }

    using DataType = typename TMatrix::DataType;
    constexpr static std::size_t GetRows() { return TMatrix::GetColumns(); }
    constexpr static std::size_t GetColumns() { return TMatrix::GetRows(); }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        return mMatrix.Get(j, i);
    }

private:
    const TMatrix& mMatrix;
};

template <typename TMatrix>
requires ConceptMatrix<TMatrix>
class ViewNegative
{
public:
    ViewNegative(const TMatrix& matrix)
        : mMatrix(matrix)
    { }

    using DataType = typename TMatrix::DataType;
    constexpr static std::size_t GetRows() { return TMatrix::GetRows(); }
    constexpr static std::size_t GetColumns() { return TMatrix::GetColumns(); }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const { return -1.0 * mMatrix.Get(i, j); }

private:
    const TMatrix& mMatrix;
};

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2> && HasSameType<TMat1, TMat2>::value //
    class ViewRowStack
{
public:
    ViewRowStack(const TMat1& mat1, const TMat2& mat2)
        : mMatrix1(mat1)
        , mMatrix2(mat2)
    {
        static_assert(TMat1::GetColumns() == TMat2::GetColumns());
    }

    using DataType = typename TMat1::DataType;
    constexpr static std::size_t GetRows() { return TMat1::GetRows() + TMat2::GetRows(); }
    constexpr static std::size_t GetColumns() { return TMat1::GetColumns(); }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        if(i < TMat1::GetRows())
            return mMatrix1.Get(i, j);
        else
            return mMatrix2.Get(i - TMat1::GetRows(), j);
    }

private:
    const TMat1& mMatrix1;
    const TMat2& mMatrix2;
};

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2> && HasSameType<TMat1, TMat2>::value //
    class ViewColumnStack
{
public:
    static_assert(std::is_same_v<TMat1::DataType, TMat2::DataType>);

    ViewColumnStack(const TMat1& mat1, const TMat2& mat2)
        : mMatrix1(mat1)
        , mMatrix2(mat2)
    {
        static_assert(TMat1::GetRows() == TMat2::GetRows()); // todo: make concept
    }

    using DataType = typename TMat1::DataType;
    constexpr static std::size_t GetRows() { return TMat1::GetRows(); }
    constexpr static std::size_t GetColumns() { return TMat1::GetColumns() + TMat2::GetColumns(); }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        if(j < TMat1::GetColumns())
            return mMatrix1.Get(i, j);
        else
            return mMatrix2.Get(i, j - TMat1::GetColumns());
    }

private:
    const TMat1& mMatrix1;
    const TMat2& mMatrix2;
};

// put several matrices along the diagonal
// This object can be nested, e.g. diag(diag(a, b), diag(c, d))
template <typename TMat1, typename TMat2>
requires ConceptSquareMatrix<TMat1> && ConceptSquareMatrix<TMat2> && HasSameType<TMat1, TMat2>::value //
    class ViewBlockDiagonal
{
public:
    ViewBlockDiagonal(const TMat1& mat1, const TMat2& mat2)
        : mMatrix1(mat1)
        , mMatrix2(mat2)
    { }

    using DataType = typename TMat1::DataType;
    constexpr static std::size_t GetRows() { return TMat1::GetRows() + TMat2::GetRows(); }
    constexpr static std::size_t GetColumns() { return TMat1::GetColumns() + TMat2::GetColumns(); }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        assert(((i < GetRows()) && (j < GetColumns())));
        constexpr const static std::size_t n = TMat1::GetColumns();
        if(i < n)
            if(j < n)
                return mMatrix1.Get(i, j);
            else
                return DataType{0};
        else if(j < n)
            return DataType{0};
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
        : mMatrix(mat)
    { }

    using DataType = typename TMat::DataType;
    constexpr static std::size_t GetRows() { return rows; }
    constexpr static std::size_t GetColumns() { return cols; }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        return mMatrix.Get(i + rowStart, j + colStart); //
    }

private:
    const TMat& mMatrix;
};

enum class TriangleType
{
    Upper,
    StrictUpper,
    Lower,
    StrictLower
};

template <TriangleType type, typename TMatrix>
requires ConceptMatrix<TMatrix>
class ViewTriangle
{
public:
    ViewTriangle(const TMatrix& matrix)
        : mMatrix(matrix)
    { }

    using DataType = typename TMatrix::DataType;
    constexpr static std::size_t GetRows() { return TMatrix::GetRows(); }
    constexpr static std::size_t GetColumns() { return TMatrix::GetColumns(); }

    constexpr DataType Get(const std::size_t i, const std::size_t j) const
    {
        if constexpr(type == TriangleType::Upper)
            return (i > j) ? 0. : mMatrix.Get(i, j);
        else if constexpr(type == TriangleType::StrictUpper)
            return (i >= j) ? 0. : mMatrix.Get(i, j);
        else if constexpr(type == TriangleType::Lower)
            return (i < j) ? 0. : mMatrix.Get(i, j);
        else // if constexpr(type == TriangleType::StrictLower)
            return (i <= j) ? 0. : mMatrix.Get(i, j);
        //else
        //{
        //    static_assert(false); // unknown enum
        //    return 0.;
        //}
    }

private:
    const TMatrix& mMatrix;
};

template <typename TMatrix>
using ViewUpperTriangle = ViewTriangle<TriangleType::Upper, TMatrix>;

template <typename TMatrix>
using ViewStrictUpperTriangle = ViewTriangle<TriangleType::StrictUpper, TMatrix>;

template <typename TMatrix>
using ViewLowerTriangle = ViewTriangle<TriangleType::Lower, TMatrix>;

template <typename TMatrix>
using ViewStrictLowerTriangle = ViewTriangle<TriangleType::StrictLower, TMatrix>;

} // namespace linalg
} // namespace vic