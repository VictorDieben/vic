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

template <typename T, typename... Ts>
using AllTs = typename std::conjunction<std::is_same<Ts, T>...>::value;

template <typename T, typename TShape>
struct MatrixConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    using MatrixBase = MatrixBaseSelector<TShape>;
    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;
    constexpr static MatrixSize ArraySize = MatrixBase::GetRows() * MatrixBase::GetColumns();

    constexpr MatrixConst() = default;
    constexpr MatrixConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    { }
    constexpr MatrixConst(const std::array<T, ArraySize>& data)
        : mData(data)
    { }

    template <typename... Ts>
        requires(std::is_convertible_v<Ts, T> && ...) && (sizeof...(Ts) == ArraySize)
    constexpr MatrixConst(Ts&&... data)
        : mData(std::forward<Ts>(data)...)
    { }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData.at(RowMayorRowColToIndex<TShape::cols>(i, j));
    }
    constexpr T Get(const Row i) const
    {
        assert(i < MatrixBase::GetRows());
        static_assert(MatrixBase::GetColumns() == 1);
        return mData.at(RowMayorRowColToIndex<TShape::cols>(i, 0));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
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
        mData.resize(MatrixBase::GetRows() * MatrixBase::GetColumns());
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData.at(ColMayorRowColToIndex<TShape::rows>(i, j));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData[ColMayorRowColToIndex<TShape::rows>(i, j)];
    }

private:
    std::vector<T> mData{};
};

template <typename T, typename TShape>
struct MatrixColConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    using MatrixBase = MatrixBaseSelector<TShape>;

    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;
    MatrixColConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        assert(TShape::cols == cols);
        mData.resize(MatrixBase::GetRows() * MatrixBase::GetColumns());
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData.at(RowMayorRowColToIndex<TShape::cols>(i, j));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData[RowMayorRowColToIndex<TShape::cols>(i, j)];
    }

private:
    std::vector<T> mData{};
};

template <typename T, typename TShape>
struct MatrixDynamic : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    using MatrixBase = MatrixBaseSelector<TShape>;

    constexpr static auto Ordering = EOrdering::RowMayor;
    constexpr static auto Distribution = EDistribution::Full;
    MatrixDynamic(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        mData.resize(MatrixBase::GetRows() * MatrixBase::GetColumns());
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData.at(RowMayorRowColToIndex(i, j, MatrixBase::GetColumns()));
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData[RowMayorRowColToIndex(i, j, MatrixBase::GetColumns())];
    }

private:
    std::vector<T> mData{};
};

} // namespace detail

template <typename T, typename TShape = Shape<UnknownSize, UnknownSize>>
using Matrix = TypeSelector<TShape, //
                            detail::MatrixConst<T, TShape>,
                            detail::MatrixRowConst<T, TShape>,
                            detail::MatrixColConst<T, TShape>,
                            detail::MatrixDynamic<T, TShape>>;

template <typename T, Row rowSize = UnknownSize>
using Vector = std::conditional_t<rowSize == UnknownSize, //
                                  detail::MatrixDynamic<T, Shape<rowSize, 1u>>,
                                  detail::MatrixColConst<T, Shape<rowSize, 1u>>>;

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

template <typename... Ts>
constexpr auto MakeVector(Ts&&... ts)
{
    using CT = std::common_type_t<Ts...>;
    std::array<CT, sizeof...(Ts)> data{std::forward<CT>(ts)...};
    return VectorN<CT, sizeof...(Ts)>(data);
}

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