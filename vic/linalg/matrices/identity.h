#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/matrices/base.h"

namespace vic
{
namespace linalg
{
namespace detail
{

template <typename T, typename TShape>
struct IdentityImpl : public MatrixBaseSelector<TShape>
{
    static_assert(TShape::rows == TShape::cols || //
                  TShape::rows == UnknownSize || //
                  TShape::cols == UnknownSize);

    using DataType = T;
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Diagonal;

    // todo: find better solution
    constexpr static bool TempIsIdentity = true;
    constexpr static bool TempIsDiagonal = true;

    constexpr IdentityImpl() = default; // todo: only allowed for constexpr size
    constexpr IdentityImpl(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    {
        assert(rows == cols);
    }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(i < this->GetRows() && j < this->GetColumns());
        return static_cast<T>(i == j);
    }
};

} // namespace detail

template <typename T, typename TShape = UnknownShape>
using Identity = detail::IdentityImpl<T, TShape>; // No need for type selector yet

// todo: do we want to allow this?
//template <typename T, Row rows, Col cols>
//using IdentityMxN = detail::IdentityImpl<T, Shape<rows, cols>>;

template <typename T, MatrixSize size>
using IdentityN = detail::IdentityImpl<T, Shape<size, size>>;

template <typename T>
using Identity1 = IdentityN<T, 1>;
template <typename T>
using Identity2 = IdentityN<T, 2>;
template <typename T>
using Identity3 = IdentityN<T, 3>;
template <typename T>
using Identity4 = IdentityN<T, 4>;
template <typename T>
using Identity5 = IdentityN<T, 5>;
template <typename T>
using Identity6 = IdentityN<T, 6>;

using Identity1d = IdentityN<double, 1>;
using Identity2d = IdentityN<double, 2>;
using Identity3d = IdentityN<double, 3>;
using Identity4d = IdentityN<double, 4>;
using Identity5d = IdentityN<double, 5>;
using Identity6d = IdentityN<double, 6>;

using Identity1f = IdentityN<float, 1>;
using Identity2f = IdentityN<float, 2>;
using Identity3f = IdentityN<float, 3>;
using Identity4f = IdentityN<float, 4>;
using Identity5f = IdentityN<float, 5>;
using Identity6f = IdentityN<float, 6>;

} // namespace linalg
} // namespace vic