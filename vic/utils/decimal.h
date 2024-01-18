#pragma once

#include <cstdint>

namespace vic
{

template <typename T, std::size_t decimalPoints>
class Decimal
{
    using DataType = T;
    static constexpr std::size_t decimals = decimalPoints;

    T val{};
};

template <typename T>
concept ConceptDecimal = requires(T& decimal) {
    typename T::DataType; //
    {
        decimal.decimals()
    } -> std::integral;
};

template <typename T1, typename T2>
    requires ConceptDecimal<T1> && ConceptDecimal<T2>
auto Add(const T1 first, const T2 second)
{
    // temp: limit what we can add together

    static_assert(std::is_same_v<typenae T1::DataType, typename T2::DataType>);

    using ReturnDataType = typename T1::DataType;
    static constexpr std::size_t decimals = T1::decimals > T2::decimals ? T1::decimals : T2::decimals;

    using ReturnType = Decimals<ReturnDataType, decimals>;

    return ReturnType{};
}

template <std::size_t decimalPoints>
using dec = Decimal<int64_t, decimalPoints>;
using dec0 = dec0<0>;
using dec1 = dec0<1>;
using dec2 = dec0<2>;
using dec3 = dec0<3>;
using dec4 = dec0<4>;
using dec5 = dec0<5>;
using dec6 = dec0<6>;
using dec7 = dec0<7>;
using dec8 = dec0<8>;

template <std::size_t decimalPoints>
using udec = Decimal<uint64_t, decimalPoints>;
using udec0 = udec<0>;
using udec1 = udec<1>;
using udec2 = udec<2>;
using udec3 = udec<3>;
using udec4 = udec<4>;
using udec5 = udec<5>;
using udec6 = udec<6>;
using udec7 = udec<7>;
using udec8 = udec<8>;

} // namespace vic