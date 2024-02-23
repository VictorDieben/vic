//#pragma once
//
//#include "to.h"
//#include <cstdint>
//
//namespace vic
//{
//
//using ExponentType = int64_t;
//
//// todo: make a generic template for any base, not just powers of 10
//template <typename T, ExponentType base10exponent>
//struct Decimal
//{
//    using ThisType = Decimal<T, base10exponent>;
//
//    explicit Decimal()
//        : val(0)
//    { }
//
//    explicit Decimal(const T decimalValue)
//        : val(decimalValue)
//    { }
//
//    template <typename TIntegral>
//        requires std::integral<TIntegral>
//    static Decimal<T, base10exponent> FromIntegral(const TIntegral value)
//    {
//        if constexpr(base10exponent == 0)
//            return ThisType{value};
//        else if constexpr(base10exponent < 0)
//        {
//            // Multiply input.val by a power of 10
//            static constexpr auto exponent = vic::Power<ExponentType>(10, -base10exponent);
//            return ThisType{value * exponent};
//        }
//        else
//        {
//            // todo: round of input to the nearest
//            static constexpr auto inverseExponent = vic::Power<ExponentType>(10, base10exponent);
//            return ThisType{value / inverseExponent};
//        }
//    }
//
//    template <typename TFloat>
//        requires std::floating_point<TFloat>
//    static Decimal<T, base10exponent> FromFloat(const TFloat value)
//    {
//        if constexpr(base10exponent == 0)
//            return ThisType{static_cast<T>(value)};
//        else if constexpr(base10exponent < 0)
//        {
//            // Multiply input.val by a power of 10
//            static constexpr auto exponent = vic::Power<ExponentType>(10, -base10exponent);
//            return ThisType{static_cast<T>(value * exponent)};
//        }
//        else
//        {
//            // todo: round of input to the nearest
//            static constexpr auto inverseExponent = vic::Power<ExponentType>(10, base10exponent);
//            return ThisType{static_cast<T>(value / inverseExponent)};
//        }
//    }
//
//    template <typename TIntegral = T>
//        requires std::integral<TIntegral>
//    TIntegral ToIntegral() const
//    {
//        if constexpr(base10exponent == 0)
//            return static_cast<TIntegral>(val);
//        else if constexpr(base10exponent < 0)
//        {
//            static constexpr auto exponent = vic::Power<ExponentType>(10, -base10exponent);
//            return static_cast<TIntegral>(val / exponent);
//        }
//        else // base10exponent > 0
//        {
//            static constexpr auto exponent = vic::Power<ExponentType>(10, base10exponent);
//            return static_cast<TIntegral>(val * exponent);
//        }
//    }
//
//    using DataType = T;
//    static constexpr ExponentType exp10 = base10exponent; // note: allowed to be negative
//
//    static constexpr ExponentType Exponent() { return exp10; }
//
//    T val{};
//};
//
//template <typename T>
//concept ConceptDecimal = requires(T& decimal) {
//    typename T::DataType; //
//    {
//        decltype(T::Exponent())(T::Exponent())
//
//    } -> std::integral;
//};
//
//template <typename TRes, typename T1, typename T2>
//    requires ConceptDecimal<TRes> && ConceptDecimal<T1> && ConceptDecimal<T2>
//TRes Add(const T1 first, const T2 second)
//{
//    // temp: limit what we can add together
//
//    // todo: make an add that figures out the return type automatically
//    //static_assert(std::is_same_v<typenae T1::DataType, typename T2::DataType>);
//    //using ReturnDataType = typename T1::DataType;
//    //static constexpr std::size_t decimals = T1::decimals > T2::decimals ? T1::decimals : T2::decimals;
//    //using ReturnType = Decimals<ReturnDataType, decimals>;
//
//    // NOTE: this does not always give the best result
//    TRes result = To<TRes>(first);
//    result.val += To<TRes>(second).val;
//
//    return result;
//}
//
////template <typename TDecimal>
////    requires ConceptDecimal<TDecimal>
////TDecimal Add(const TDecimal first, const TDecimal second)
////{
////    return Add<TDecimal>(first, second);
////}
//
//template <int64_t base10exponent>
//using dec = Decimal<int64_t, base10exponent>;
//using dec0 = dec<0>;
//using dec1 = dec<-1>;
//using dec2 = dec<-2>;
//using dec3 = dec<-3>;
//using dec4 = dec<-4>;
//using dec5 = dec<-5>;
//using dec6 = dec<-6>;
//using dec7 = dec<-7>;
//using dec8 = dec<-8>;
//
//// todo: name,
//using exp0 = dec<0>;
//using exp1 = dec<1>;
//using exp2 = dec<2>;
//using exp3 = dec<3>;
//using exp4 = dec<4>;
//using exp5 = dec<5>;
//using exp6 = dec<6>;
//using exp7 = dec<7>;
//using exp8 = dec<8>;
//
//template <int64_t base10exponent>
//using udec = Decimal<uint64_t, base10exponent>;
//using udec0 = udec<0>;
//using udec1 = udec<-1>;
//using udec2 = udec<-2>;
//using udec3 = udec<-3>;
//using udec4 = udec<-4>;
//using udec5 = udec<-5>;
//using udec6 = udec<-6>;
//using udec7 = udec<-7>;
//using udec8 = udec<-8>;
//
//using uexp0 = udec<0>;
//using uexp1 = udec<1>;
//using uexp2 = udec<2>;
//using uexp3 = udec<3>;
//using uexp4 = udec<4>;
//using uexp5 = udec<5>;
//using uexp6 = udec<6>;
//using uexp7 = udec<7>;
//using uexp8 = udec<8>;
//
//template <class TResult, class TInput>
//    requires ConceptDecimal<TResult> && ConceptDecimal<TInput>
//TResult To(const TInput& input)
//{
//    if constexpr(TResult::exp10 == TInput::exp10)
//        return input;
//    else if constexpr(TResult::exp10 < TInput::exp10)
//    {
//        const ExponentType p = TInput::exp10 - TResult::exp10;
//        const auto exponent = vic::Power<typename TResult::DataType>(10, p); // todo: constexpr once finished
//        return TResult{input.val * exponent};
//    }
//    else // TResult::exp10 > TInput::exp10
//    {
//        const ExponentType inverseP = TResult::exp10 - TInput::exp10;
//        const auto inverseExponent = vic::Power<typename TResult::DataType>(10, inverseP); // todo: constexpr once finished
//        return TResult{input.val / inverseExponent}; // NOTE: for now just floors, do we want to round?
//    }
//}
//
//} // namespace vic