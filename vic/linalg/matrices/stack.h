#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/index.h"
#include "vic/linalg/matrices/base.h"

#include "vic/utils.h"

#include <array>
#include <cassert>
#include <vector>

namespace vic
{
namespace linalg
{

namespace detail
{

constexpr Row GetRowStackRows(const Row row1, const Row row2)
{
    if(row1 == UnknownSize || row2 == UnknownSize)
        return UnknownSize;
    return row1 + row2;
}

template <typename TShape1, typename TShape2>
using RowStackResultShape = Shape<GetRowStackRows(TShape1::rows, TShape2::rows), Min(TShape1::cols, TShape2::cols)>;

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
struct RowStackImpl : public MatrixBaseSelector<typename RowStackResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>>
{
    using DataType = TMat1::DataType;
    using MatrixBase = MatrixBaseSelector<typename RowStackResultShape<typename TMat1::ShapeType, typename TMat2::ShapeType>>;
    constexpr static bool TempIsRowStack = true;

    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Full; //

    constexpr RowStackImpl(const TMat1& mat1, const TMat2& mat2)
        : mMat1(mat1)
        , mMat2(mat2)
    {
        // assert(mat1.GetColumns() == mat2.GetColumns());
    }

    constexpr DataType Get(const Row i, const Col j) const
    {
        assert(i < MatrixBase::GetRows() && j < MatrixBase::GetColumns());
        if(i < mMat1.GetRows())
            return mMat1.Get(i, j);
        else
            return mMat2.Get(i - mMat1.GetRows(), j);
    }

    // todo: at should only be available if either submatrix has it
    constexpr DataType& At(const Row i, const Col j)
    {
        assert(i < MatrixBase::GetRows() && j < MatrixBase::GetColumns());
        if(i < mMat1.GetRows())
            return mMat1.At(i, j);
        else
            return mMat2.At(i - mMat1.GetRows(), j);
    }

    TMat1 mMat1; // top
    TMat2 mMat2; // bottom
};

} // namespace detail

template <typename TMat1, typename TMat2>
using RowStack = detail::RowStackImpl<TMat1, TMat2>;

template <typename TMat1, typename TMat2>
requires ConceptMatrix<TMat1> && ConceptMatrix<TMat2>
constexpr auto ToRowStack(const TMat1& mat1, const TMat2& mat2)
{
    return RowStack<TMat1, TMat2>{mat1, mat2}; //
}

} // namespace linalg
} // namespace vic