#pragma once

#include <cmath>

namespace vic
{
namespace ml
{

template <typename T>
T Sigmoid(const T x)
{
    return 1. / (1. + std::exp(-x));
}

} // namespace ml
} // namespace vic