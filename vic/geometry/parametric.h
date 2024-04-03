#pragma once

#include <array>
#include <cstddef>

#include "vic/linalg/matrices/matrix.h"

namespace vic
{
namespace geom
{
// todo:
// https://www.youtube.com/watch?v=jvPPXbo87ds&t=446s

// bezier curve, 4 control points, cubic polynomial
template <typename T, std::size_t dims>
struct BCurve
{
public:
    using ControlPoint = linalg::VectorN<T, dims>;
    std::array<ControlPoint, 4> mControlPoints{};
};

// bezier curve, 4 control points, cubic polynomial
template <typename T, std::size_t dims>
struct BSurface
{
public:
    using ControlPoint = linalg::VectorN<T, dims>;
    std::array<ControlPoint, 4> mControlPoints{};
};

// Bezier Spline, any number of control points, variable order
template <typename T, std::size_t dims>
struct BSpline
{ };

enum class EDomain
{
    N, // natural numbers: 1, 2, 3, ...
    Z, // integers: -3, -2, -1, 0, 1, 2, 3, ...
    Q, // Rational Numbers: a/b
    R, // Real numbers
    Irrational, // Irrational (e, pi, ...)
    I, // Imaginary numbers
    C, // Complex numbers
    H, // Hypercomplex numbers (quaternions, octonions, etc.)
    Padic // P-adic, numbers with different limits at infinity (modulus)
};

template <EDomain domainType, uint32_t dimensionality>
struct Domain
{
    constexpr static EDomain Type = domainType;
    constexpr static uint32_t dimensions = dimensionality;
};

using Z1 = Domain<EDomain::Z, 1>;
using Z2 = Domain<EDomain::Z, 2>;
using Z3 = Domain<EDomain::Z, 3>;

using R1 = Domain<EDomain::R, 1>;
using R2 = Domain<EDomain::R, 2>;
using R3 = Domain<EDomain::R, 3>;

template <typename TDomain, typename TCoDomain>
struct Map
{
    using Domain = TDomain; // "from"
    using CoDomain = TCoDomain; // "to"
};

using MapR1R1 = Map<R1, R1>;
using MapR1R2 = Map<R1, R2>;
using MapR1R3 = Map<R1, R3>;

using MapR2R1 = Map<R2, R1>;
using MapR2R2 = Map<R2, R2>;
using MapR2R3 = Map<R2, R3>;

using MapR3R1 = Map<R3, R1>;
using MapR3R2 = Map<R3, R2>;
using MapR3R3 = Map<R3, R3>;

template <uint32_t continuity>
struct Continuity
{
    constexpr static uint32_t Value = continuity;
};

using C0 = Continuity<0>;
using C1 = Continuity<1>;
using C2 = Continuity<2>;

} // namespace geom
} // namespace vic