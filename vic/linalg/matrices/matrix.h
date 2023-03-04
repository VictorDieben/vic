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

template <typename T, typename TShape>
struct MatrixConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    using MatrixBase = MatrixBaseSelector<TShape>;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;
    constexpr static MatrixSize ArraySize = MatrixBase::GetRows() * MatrixBase::GetColumns();

    MatrixConst() = default;
    constexpr MatrixConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    { }
    constexpr MatrixConst(const std::array<T, ArraySize>& data)
        : mData(data)
    { }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData.at(RowMayorRowColToIndex<TShape::cols>(i, j));
    }
    constexpr T Get(const Row i) const
    {
        assert(i < this->GetRows());
        static_assert(this->GetColumns() == 1);
        return mData.at(RowMayorRowColToIndex<TShape::cols>(i, 0));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData[RowMayorRowColToIndex<TShape::cols>(i, j)];
    }

private:
    std::array<T, ArraySize> mData{};
};

template <typename T, typename TShape>
struct MatrixRowConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    using MatrixBase = MatrixBaseSelector<TShape>;
    constexpr static auto Ordering = EOrdering::ColumnMayor;
    constexpr static auto Distribution = EDistribution::Full;
    MatrixRowConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        assert(TShape::rows == rows);
        mData.resize(this->GetRows() * this->GetColumns());
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData.at(ColMayorRowColToIndex<TShape::rows>(i, j));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData[ColMayorRowColToIndex<TShape::rows>(i, j)];
    }

private:
    std::vector<T> mData{};
};

template <typename T, typename TShape>
struct MatrixColConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;
    MatrixColConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        assert(TShape::cols == cols);
        mData.resize(this->GetRows() * this->GetColumns());
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData.at(RowMayorRowColToIndex<TShape::cols>(i, j));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData[RowMayorRowColToIndex<TShape::cols>(i, j)];
    }

private:
    std::vector<T> mData{};
};

template <typename T, typename TShape>
struct MatrixDynamic : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;
    MatrixDynamic(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        mData.resize(this->GetRows() * this->GetColumns());
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData.at(RowMayorRowColToIndex(i, j, this->GetColumns()));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return mData[RowMayorRowColToIndex(i, j, this->GetColumns())];
    }

private:
    std::vector<T> mData{};
};

} // namespace detail

template <typename T, typename TShape>
using Matrix = TypeSelector<TShape, //
                            detail::MatrixConst<T, TShape>,
                            detail::MatrixRowConst<T, TShape>,
                            detail::MatrixColConst<T, TShape>,
                            detail::MatrixDynamic<T, TShape>>;

template <typename T, Row rows, Col cols>
using MatrixMxN = detail::MatrixConst<T, Shape<rows, cols>>;

template <typename T>
using Matrix1 = MatrixMxN<T, 1, 1>;
template <typename T>
using Matrix2 = MatrixMxN<T, 2, 2>;
template <typename T>
using Matrix3 = MatrixMxN<T, 3, 3>;
template <typename T>
using Matrix4 = MatrixMxN<T, 4, 4>;
template <typename T>
using Matrix5 = MatrixMxN<T, 5, 5>;
template <typename T>
using Matrix6 = MatrixMxN<T, 6, 6>;

template <typename T, Row rows>
using VectorN = MatrixMxN<T, rows, 1>;

template <typename T>
using Vector1 = VectorN<T, 1>;
template <typename T>
using Vector2 = VectorN<T, 2>;
template <typename T>
using Vector3 = VectorN<T, 3>;
template <typename T>
using Vector4 = VectorN<T, 4>;
template <typename T>
using Vector5 = VectorN<T, 5>;
template <typename T>
using Vector6 = VectorN<T, 6>;

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto ToFull(const TMat& mat)
{
    Matrix<typename TMat::DataType, typename TMat::ShapeType> res{mat.GetRows(), mat.GetColumns()};

    for(Row i = 0; i < mat.GetRows(); ++i)
        for(Col j = 0; j < mat.GetColumns(); ++j)
            res.At(i, j) = mat.Get(i, j);

    return res;
}

} // namespace linalg
} // namespace vic