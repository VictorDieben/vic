#pragma once

#include "vic/linalg/definitions.h"
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
struct DiagonalConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Diagonal;

    constexpr static MatrixSize DiagSize = Min(TShape::rows, TShape::cols);

    constexpr static bool TempIsDiagonal = true; // todo: find better solution

    DiagonalConst() = default;
    constexpr DiagonalConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(TShape::rows, TShape::cols)
    {
        assert(TShape::rows == rows && TShape::cols == cols);
    }
    constexpr DiagonalConst(const std::array<T, DiagSize>& data)
        : MatrixBaseSelector<TShape>(TShape::rows, TShape::cols)
        , mData(data)
    { }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return i == j ? mData.at(i) : T{0};
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        assert(i == j); // only diagonal can be set
        return mData[i];
    }

private:
    std::array<T, DiagSize> mData{};
};

template <typename T, typename TShape>
struct DiagonalRowConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Diagonal;

    constexpr static bool TempIsDiagonal = true; // todo: find better solution

    explicit DiagonalRowConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        assert(TShape::rows == rows);
        mData.resize(Min(TShape::rows, cols));
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return i == j ? mData.at(i) : T{0};
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        assert(i == j); // only diagonal can be set
        return mData[i];
    }

private:
    std::vector<T> mData{};
};

template <typename T, typename TShape>
struct DiagonalColConst : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Diagonal;

    constexpr static bool TempIsDiagonal = true; // todo: find better solution

    explicit DiagonalColConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        assert(TShape::cols == cols);
        mData.resize(Min(TShape::cols, rows));
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return i == j ? mData.at(i) : T{0};
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        assert(i == j); // only diagonal can be set
        return mData[i];
    }

private:
    std::vector<T> mData{};
};

template <typename T, typename TShape>
struct DiagonalDynamic : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Diagonal;

    constexpr static bool TempIsDiagonal = true; // todo: find better solution

    explicit DiagonalDynamic(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        mData.resize(Min(rows, cols));
    }
    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        return i == j ? mData.at(i) : T{0};
    }
    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));
        assert(i == j); // only diagonal can be set
        return mData[i];
    }

private:
    std::vector<T> mData{};
};

} // namespace detail

template <typename T, typename TShape = Shape<UnknownSize, UnknownSize>>
using Diagonal = TypeSelector<TShape, //
                              detail::DiagonalConst<T, TShape>,
                              detail::DiagonalRowConst<T, TShape>,
                              detail::DiagonalColConst<T, TShape>,
                              detail::DiagonalDynamic<T, TShape>>;

template <typename T, Row rows, Col cols>
using DiagonalMxN = detail::DiagonalConst<T, Shape<rows, cols>>;

template <typename T>
using Diagonal1 = DiagonalMxN<T, 1, 1>;
template <typename T>
using Diagonal2 = DiagonalMxN<T, 2, 2>;
template <typename T>
using Diagonal3 = DiagonalMxN<T, 3, 3>;
template <typename T>
using Diagonal4 = DiagonalMxN<T, 4, 4>;
template <typename T>
using Diagonal5 = DiagonalMxN<T, 5, 5>;
template <typename T>
using Diagonal6 = DiagonalMxN<T, 6, 6>;

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto ToDiagonal(const TMat& mat)
{
    Diagonal<typename TMat::DataType, typename TMat::ShapeType> res{mat.GetRows(), mat.GetColumns()};
    for(MatrixSize i = 0; i < Min(mat.GetRows(), mat.GetColumns()); ++i)
        res.At(i, i) = mat.Get(i, i);
    return res;
}

template <typename T, std::size_t n>
constexpr auto ToDiagonal(const std::array<T, n>& data)
{
    DiagonalMxN<T, n, n> res{};
    for(MatrixSize i = 0; i < n; ++i)
        res.At(i, i) = data.at(i);
    return res;
}

} // namespace linalg
} // namespace vic