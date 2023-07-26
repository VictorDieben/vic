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
struct OnesImpl : public MatrixBaseSelector<TShape>
{
    using DataType = T;
    constexpr static auto Ordering = EOrdering::Any;
    constexpr static auto Distribution = EDistribution::Unknown;

    constexpr OnesImpl() = default;
    constexpr OnesImpl(const Row rows, const Col cols)
        : MatrixBaseSelector<TShape>(rows, cols)
    { }

    constexpr T Get(const Row i, const Col j) const
    {
        assert(i < this->GetRows() && j < this->GetColumns());
        return T{1};
    }
};

} // namespace detail

template <typename T, typename TShape = Shape<UnknownSize, UnknownSize>>
using Ones = detail::OnesImpl<T, TShape>; // No need for type selector yet

template <typename T, Row rows, Col cols>
using OnesMxN = detail::OnesImpl<T, Shape<rows, cols>>;

template <typename T, MatrixSize size>
using OnesN = detail::OnesImpl<T, Shape<size, size>>;

template <typename T>
using Ones1 = OnesMxN<T, 1, 1>;
template <typename T>
using Ones2 = OnesMxN<T, 2, 2>;
template <typename T>
using Ones3 = OnesMxN<T, 3, 3>;
template <typename T>
using Ones4 = OnesMxN<T, 4, 4>;
template <typename T>
using Ones5 = OnesMxN<T, 5, 5>;
template <typename T>
using Ones6 = OnesMxN<T, 6, 6>;

template <typename T, Row rows>
using OnesVectorN = OnesMxN<T, rows, 1>;

template <typename T>
using OnesVector1 = OnesVectorN<T, 1>;
template <typename T>
using OnesVector2 = OnesVectorN<T, 2>;
template <typename T>
using OnesVector3 = OnesVectorN<T, 3>;
template <typename T>
using OnesVector4 = OnesVectorN<T, 4>;
template <typename T>
using OnesVector5 = OnesVectorN<T, 5>;
template <typename T>
using OnesVector6 = OnesVectorN<T, 6>;

template <typename TMat>
requires ConceptMatrix<TMat>
constexpr auto ToOnes(const TMat& mat)
{
    return Ones<typename TMat::DataType, typename TMat::ShapeType>{mat.GetRows(), mat.GetColumns()}; //
}

} // namespace linalg
} // namespace vic