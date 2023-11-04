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

    constexpr T Get(const Row i, const Col j) const noexcept
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData[RowMayorRowColToIndex<TShape::cols>(i, j)];
    }
    constexpr T Get(const Row i) const noexcept
    {
        assert(i < MatrixBase::GetRows());
        static_assert(MatrixBase::GetColumns() == 1);
        return mData[RowMayorRowColToIndex<TShape::cols>(i, 0)];
    }
    constexpr T& At(const Row i, const Col j) noexcept
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
    constexpr T Get(const Row i, const Col j) const noexcept
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData[ColMayorRowColToIndex<TShape::rows>(i, j)];
    }
    constexpr T& At(const Row i, const Col j) noexcept
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
    constexpr T Get(const Row i, const Col j) const noexcept
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData[RowMayorRowColToIndex<TShape::cols>(i, j)];
    }
    constexpr T& At(const Row i, const Col j) noexcept
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
    constexpr T Get(const Row i, const Col j) const noexcept
    {
        assert(((i < MatrixBase::GetRows()) && (j < MatrixBase::GetColumns())));
        return mData[RowMayorRowColToIndex(i, j, MatrixBase::GetColumns())];
    }
    constexpr T& At(const Row i, const Col j) noexcept
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

using Matrix11f = MatrixMxN<float, 1, 1>;
using Matrix22f = MatrixMxN<float, 2, 3>;
using Matrix33f = MatrixMxN<float, 3, 3>;
using Matrix44f = MatrixMxN<float, 4, 4>;
using Matrix55f = MatrixMxN<float, 5, 5>;
using Matrix66f = MatrixMxN<float, 6, 6>;

using Matrix11d = MatrixMxN<double, 1, 1>;
using Matrix22d = MatrixMxN<double, 2, 3>;
using Matrix33d = MatrixMxN<double, 3, 3>;
using Matrix44d = MatrixMxN<double, 4, 4>;
using Matrix55d = MatrixMxN<double, 5, 5>;
using Matrix66d = MatrixMxN<double, 6, 6>;

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

using Vector1f = VectorN<float, 1>;
using Vector2f = VectorN<float, 2>;
using Vector3f = VectorN<float, 3>;
using Vector4f = VectorN<float, 4>;
using Vector5f = VectorN<float, 5>;
using Vector6f = VectorN<float, 6>;

using Vector1d = VectorN<double, 1>;
using Vector2d = VectorN<double, 2>;
using Vector3d = VectorN<double, 3>;
using Vector4d = VectorN<double, 4>;
using Vector5d = VectorN<double, 5>;
using Vector6d = VectorN<double, 6>;

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