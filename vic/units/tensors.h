#pragma once

#include "assert.h"

#include "vic/linalg/matrices.h"
#include "vic/utils.h"

#include <array>
#include <cstddef>

// TODO: should i make this a separate module?
// It will likely contain similar components as units,
// but the mathematical stuff will be far more involved,
// and needs proper testing.

// khan university video:
// https://www.youtube.com/watch?v=BaIMIRmUtLs

namespace vic
{
namespace tensor
{
template <std::size_t dims>
struct Space
{
    constexpr static std::size_t Dims = dims;
};

template <typename TSpace, typename T>
struct Basis
{
    using Dims = typename TSpace::Dims;
    linalg::Matrix<T, Dims, Dims> basis; // basis[1/2/3][x/y/z]
};

// todo: contravariant vector; a^i

// todo: covariant vector: b_j

// inner product

// outer product (kroniker product/tensor product)

//

} // namespace tensor
} // namespace vic