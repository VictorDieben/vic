#pragma once

#include "vic/linalg/matrices/base.h"
#include "vic/linalg/tools.h"
#include "vic/linalg/traits.h"

#include <array>
#include <vector>

namespace vic
{
namespace linalg
{
namespace detail
{
// [b1 c1 0 ]
// [a1 b2 c2]
// [0  a2 b3]

template <typename T, typename TShape>
class TriDiagonalConst : public MatrixBaseSelector<TShape>
{
public:
    static_assert(TShape::rows == TShape::cols);

    using DataType = T;
    constexpr static auto Ordering = EOrdering::Custom;
    constexpr static auto Distribution = EDistribution::TriDiagonal;

    constexpr static bool TempIsTriDiagonal = true;

    constexpr TriDiagonalConst(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(TShape::rows, TShape::cols)
    {
        assert(TShape::rows == rows && TShape::cols == cols);
    }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));

        if(i == j)
            return mB[i];
        else if(i + 1 == j)
            return mC[i];
        else if(i == j + 1)
            return mA[j];
        else
            return T{0};
    }

    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));

        if(i + 1 == j)
            return mC[i];
        else if(i == j + 1)
            return mA[j];

        // weird structure so that all paths return a value,
        // but this should only happen if  i==j
        assert(i == j);
        return mB[i];
    }

    constexpr const auto& A() const { return mA; }
    constexpr const auto& B() const { return mB; }
    constexpr const auto& C() const { return mC; }

private:
    static constexpr MatrixSize mDiagonalSize = std::min(TShape::rows, TShape::cols);
    static constexpr MatrixSize mOffDiagonalSize = mDiagonalSize - 1;

    std::array<T, mOffDiagonalSize> mA{};
    std::array<T, mDiagonalSize> mB{};
    std::array<T, mOffDiagonalSize> mC{};
};

template <typename T, typename TShape>
class TriDiagonalDyn : public MatrixBaseSelector<TShape>
{
public:
    static_assert(TShape::rows == TShape::cols);

    using DataType = T;
    constexpr static auto Ordering = EOrdering::Custom;
    constexpr static auto Distribution = EDistribution::TriDiagonal;

    constexpr static bool TempIsTriDiagonal = true;

    constexpr TriDiagonalDyn(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
        , mA(rows == 0 ? 0 : rows - 1, 0.)
        , mB(rows, 0.)
        , mC(rows == 0 ? 0 : rows - 1, 0.)
    {
        assert(rows == cols);
    }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));

        if(i == j)
            return mB[i];
        else if(i + 1 == j)
            return mC[i];
        else if(i == j + 1)
            return mA[j];
        else
            return T{0};
    }

    constexpr T& At(const Row i, const Col j)
    {
        assert(((i < this->GetRows()) && (j < this->GetColumns())));

        if(i + 1 == j)
            return mC[i];
        else if(i == j + 1)
            return mA[j];

        // weird structure so that all paths return a value,
        // but this should only happen if  i==j
        assert(i == j);
        return mB[i];
    }

    const auto& A() const { return mA; }
    const auto& B() const { return mB; }
    const auto& C() const { return mC; }

private:
    // todo: give the user access to these objects,
    // it can be very useful to iterate over them directly
    std::vector<T> mA{};
    std::vector<T> mB{};
    std::vector<T> mC{};
};

} // namespace detail

template <typename T, typename TShape = Shape<UnknownSize, UnknownSize>>
using TriDiagonal = TypeSelector<TShape, //
                                 detail::TriDiagonalConst<T, SquareShape<TShape>>,
                                 detail::TriDiagonalConst<T, SquareShape<TShape>>,
                                 detail::TriDiagonalConst<T, SquareShape<TShape>>,
                                 detail::TriDiagonalDyn<T, TShape>>;

template <typename T, MatrixSize size>
using TriDiagonalN = detail::TriDiagonalConst<T, Shape<size, size>>;

template <typename T>
using TriDiagonal1 = TriDiagonalN<T, 1>;
template <typename T>
using TriDiagonal2 = TriDiagonalN<T, 2>;
template <typename T>
using TriDiagonal3 = TriDiagonalN<T, 3>;
template <typename T>
using TriDiagonal4 = TriDiagonalN<T, 4>;
template <typename T>
using TriDiagonal5 = TriDiagonalN<T, 5>;
template <typename T>
using TriDiagonal6 = TriDiagonalN<T, 6>;

template <typename TMat>
    requires ConceptMatrix<TMat>
constexpr auto ToTriDiagonal(const TMat& mat)
{
    TriDiagonal<typename TMat::DataType, typename TMat::ShapeType> res{mat.GetRows(), mat.GetColumns()};

    const auto diagSize = Min(mat.GetRows(), mat.GetColumns());
    const auto offDiagSize = diagSize == 0 ? 0 : diagSize - 1;
    // copy diagonal
    for(MatrixSize i = 0; i < diagSize; ++i)
        res.At(i, i) = mat.Get(i, i);

    // lower off-diagonal
    for(MatrixSize i = 0; i < offDiagSize; ++i)
        res.At(i + 1, i) = mat.Get(i + 1, i);

    // upper off-diagonal
    for(MatrixSize i = 0; i < offDiagSize; ++i)
        res.At(i, i + 1) = mat.Get(i, i + 1);

    return res;
}

} // namespace linalg
} // namespace vic