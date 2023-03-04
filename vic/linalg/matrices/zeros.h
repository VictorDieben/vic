#pragma once

#include "vic/linalg/definitions.h"
#include "vic/linalg/matrices/base.h"

#include <cassert>

namespace vic
{
namespace linalg
{
namespace detail
{

template <typename T, typename TShape>
struct ZerosImpl : public MatrixBaseSelector<T, TShape>
{
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Unknown;

    constexpr static bool TempIsZeros = true; // todo: find better solution

    constexpr ZerosImpl() = default;
    constexpr ZerosImpl(const Row rows, const Col cols)
        : MatrixBaseSelector<T, TShape>(rows, cols)
    { }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(i < this->GetRows() && j < this->GetColumns());
        return T{0};
    }
};

} // namespace detail

template <typename T, typename TShape>
using Zeros = detail::ZerosImpl<T, TShape>; // No need for type selector yet

template <typename T, Row rows, Col cols>
using ZerosMxN = detail::ZerosImpl<T, Shape<rows, cols>>;

template <typename T, MatrixSize size>
using ZerosN = detail::ZerosImpl<T, Shape<size, size>>;

template <typename T>
using Zeros1 = ZerosMxN<T, 1, 1>;
template <typename T>
using Zeros2 = ZerosMxN<T, 2, 2>;
template <typename T>
using Zeros3 = ZerosMxN<T, 3, 3>;
template <typename T>
using Zeros4 = ZerosMxN<T, 4, 4>;
template <typename T>
using Zeros5 = ZerosMxN<T, 5, 5>;
template <typename T>
using Zeros6 = ZerosMxN<T, 6, 6>;

template <typename T, Row rows>
using ZerosVectorN = ZerosMxN<T, rows, 1>;

template <typename T>
using ZerosVector1 = ZerosVectorN<T, 1>;
template <typename T>
using ZerosVector2 = ZerosVectorN<T, 2>;
template <typename T>
using ZerosVector3 = ZerosVectorN<T, 3>;
template <typename T>
using ZerosVector4 = ZerosVectorN<T, 4>;
template <typename T>
using ZerosVector5 = ZerosVectorN<T, 5>;
template <typename T>
using ZerosVector6 = ZerosVectorN<T, 6>;

} // namespace linalg
} // namespace vic