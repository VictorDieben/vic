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
struct IdentityImpl : public MatrixBaseSelector<T, TShape>
{
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Diagonal;

    constexpr static bool TempIsIdentity = true; // todo: find better solution

    constexpr IdentityImpl() = default;
    constexpr IdentityImpl(const Row rows, const Col cols)
        : MatrixBaseSelector<T, TShape>(rows, cols)
    { }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(i < GetRows() && j < GetColumns());
        return static_cast<T>(i == j);
    }
};

} // namespace detail

template <typename T, typename TShape>
using Identity = detail::IdentityImpl<T, TShape>; // No need for type selector yet

template <typename T, Row rows, Col cols>
using IdentityMxN = detail::IdentityImpl<T, Shape<rows, cols>>;

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

} // namespace linalg
} // namespace vic