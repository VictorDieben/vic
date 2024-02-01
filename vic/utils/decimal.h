#pragma once

#include <cstdint>

namespace vic
{

namespace decimal
{

using ExponentType = int64_t;

// todo: make a generic template for any base, not just 10
template <typename T, ExponentType base10exponent>
struct Decimal
{
    using DataType = T;
    static constexpr ExponentType exp10 = base10exponent; // note: allowed to be negative

    static constexpr ExponentType Exponent() { return exp10; }

    T val{};
};

template <typename T>
concept ConceptDecimal = requires(T& decimal) {
    typename T::DataType; //
    {
        T::Exponent()
    } -> std::integral;
};

template <typename TResult, typename TInput>
    requires ConceptDecimal<TResult> && ConceptDecimal<TInput>
TResult To(TInput input)
{
    return TResult{};
}

template <typename TRes, typename T1, typename T2>
    requires ConceptDecimal<TRes> && ConceptDecimal<T1> && ConceptDecimal<T2>
TRes Add(const T1 first, const T2 second)
{
    // temp: limit what we can add together

    // todo: make an add that figures out the return type automatically
    //static_assert(std::is_same_v<typenae T1::DataType, typename T2::DataType>);
    //using ReturnDataType = typename T1::DataType;
    //static constexpr std::size_t decimals = T1::decimals > T2::decimals ? T1::decimals : T2::decimals;
    //using ReturnType = Decimals<ReturnDataType, decimals>;

    // NOTE: this does not always give the best result
    TRes result = To<TRes>(first);
    result.val += To<TRes>(second).val;

    static constexpr ExponentType lowestCommon = std::max(T1::exp10, T2::exp10);

    return result;
}

template <int64_t base10exponent>
using dec = Decimal<int64_t, base10exponent>;
using dec0 = dec<0>;
using dec1 = dec<-1>;
using dec2 = dec<-2>;
using dec3 = dec<-3>;
using dec4 = dec<-4>;
using dec5 = dec<-5>;
using dec6 = dec<-6>;
using dec7 = dec<-7>;
using dec8 = dec<-8>;

template <int64_t base10exponent>
using udec = Decimal<uint64_t, base10exponent>;
using udec0 = udec<0>;
using udec1 = udec<-1>;
using udec2 = udec<-2>;
using udec3 = udec<-3>;
using udec4 = udec<-4>;
using udec5 = udec<-5>;
using udec6 = udec<-6>;
using udec7 = udec<-7>;
using udec8 = udec<-8>;

} // namespace decimal
} // namespace vic