#pragma once

#include <tuple>

namespace vic
{

template <typename TDerivativeFunctor, typename TIntegrateFunctor, typename TState, typename T>
TState ForwardEuler(const TState& state0, //
                    const T dt,
                    const TDerivativeFunctor derivative,
                    const TIntegrateFunctor integrate)
{
    const auto dot = derivative(state0);
    const auto state1 = integrate(state0, dot, dt);
    return state1;
}

template <typename TDerivativeFunctor, typename TIntegrateFunctor, typename TState, typename T>
TState BackwardEuler(const TState& state0, //
                     const T dt,
                     const TDerivativeFunctor derivative,
                     const TIntegrateFunctor integrate)
{
    // forward euler step
    auto dot1 = derivative(state0);
    auto state1 = integrate(state0, dot1, dt);

    // iterative backward euler steps
    for(std::size_t i = 0; i < 10; ++i)
    {
        auto dot1 = derivative(state1);
        auto state1 = integrate(state0, dot1, dt);
    }
    return state1;
}

template <typename TDerivativeFunctor, typename TIntegrateFunctor, typename TState, typename T>
auto Heun(const TState& state0, //
          const T dt,
          const TDerivativeFunctor derivative,
          const TIntegrateFunctor integrate)
{
    // Explicit trapezoidal/RK2
    const auto dot0 = derivative(state0);
    const auto state1_ = integrate(state0, dot0, dt);
    const auto dot1 = derivative(state1_);
    const auto state1 = integrate(state0, 0.5 * (dot0 + dot1), dt);
    return state1;
}

template <typename TDerivativeFunctor, typename TIntegrateFunctor, typename TState, typename T>
auto Trapezoidal(const TState& state0, //
                 const T dt,
                 const TDerivativeFunctor derivative,
                 const TIntegrateFunctor integrate) // todo: optional end condition
{
    const auto dot0 = derivative(state0);
    auto state1 = integrate(state0, dot0, dt); // forward euler step
    typename std::remove_const<decltype(dot0)>::type dot1;

    for(std::size_t i = 0; i < 10; ++i) // iteratively update state at t=1
    {
        dot1 = derivative(state1);
        state1 = integrate(state0, (dot0 + dot1), dt / 2.);
    }

    return state1;
}

template <typename TDerivativeFunctor, typename TIntegrateFunctor, typename TState, typename T>
TState RungeKutta4(const TState& state0, //
                   const T dt,
                   const TDerivativeFunctor derivative,
                   const TIntegrateFunctor integrate)
{
    const auto d0 = derivative(state0); // k1
    const auto s05 = integrate(state0, d0, dt / 2.);
    const auto d05 = derivative(s05); // k2
    const auto s05_2 = integrate(state0, d05, dt / 2.);
    const auto d05_2 = derivative(s05_2); // k3
    const auto s1 = integrate(state0, d05_2, dt);
    const auto d1 = derivative(s1); // k4

    // return state0 + ((dt / 6.) * (d0 + (2. * d05) + (2. * d05_2) + d1));
    return integrate(state0, d0 + (2. * d05) + (2. * d05_2) + d1, dt / 6.);
}

} // namespace vic