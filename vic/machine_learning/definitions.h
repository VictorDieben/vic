#pragma once

#include <cmath>

namespace vic
{
namespace ml
{
template <typename T>
T ReLUFunction(const T x)
{
    return std::max(0., x);
}

template <typename T>
T SigmoidFunction(const T x)
{
    return 1. / (1. + std::exp(-x));
}

template <typename T>
concept ConcepActivationFunction = requires(T object)
{
    typename T::DataType;
    T::eval(1.);
    T::eval_dydx(1.);
};

template <typename T>
struct Sigmoid
{
    using DataType = T;
    static constexpr T eval(const T x) { return SigmoidFunction(x); }
    static constexpr T eval_dydx(const T x)
    {
        const T sigmoid = SigmoidFunction(x);
        return sigmoid * (1. - sigmoid);
    }
};

template <typename T>
struct ReLU
{
    using DataType = T;
    static constexpr T eval(const T x) { return x >= 0 ? x : 0.; } // note: std::max is not constexpr
    static constexpr T eval_dydx(const T x) { return x >= 0. ? 1. : 0; }
};

static_assert(ConcepActivationFunction<Sigmoid<double>>);
static_assert(ConcepActivationFunction<ReLU<double>>);

} // namespace ml
} // namespace vic