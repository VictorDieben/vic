#pragma once

#include <array>
#include <cstddef>
#include <numeric>
#include <ranges>

namespace vic
{
namespace math
{

constexpr std::size_t Factorial(const std::size_t n)
{
    if(n == 1)
        return 1;
    else
        return n * Factorial(n - 1);
}

template <typename T>
struct factorial_info;

template <>
struct factorial_info<float>
{
    constexpr static std::size_t limit = 170;
};

template <>
struct factorial_info<double>
{
    constexpr static std::size_t limit = 170;
};

// contains constexpr versions of cmath functions
namespace detail
{

template <typename T, std::size_t n, double f>
inline constexpr T exp_recursive(const T x, const T p) noexcept
{
    // p = x^n
    // e^x = 1 + x/1! + x^2/2! + x^3/3! + ...

    // todo: decide end condition based on some rationale
    // todo: split in a x > 1 and x < 1 function? that way only one check is needed
    if constexpr(n > 1000 || f > 1.e307)
        return 0.;
    else
    {
        if(p > 1.e307 || p < 1.e-307)
            return 0.;
        static constexpr double oneOverF = 1. / f;
        const T iter = p * oneOverF;
        const T recursive = exp_recursive<T, n + 1, f * n>(x, p * x);
        return iter + recursive;
    }
}

} // namespace detail

template <typename T>
constexpr T exp(const T x) noexcept
{
    // todo: good way of defining max recursions
    // we need to calculate the maximum power of x before it gets to close to infinity
    // return detail::exp_recursive<T, 0, 100>(x, 1.);
    return detail::exp_recursive<T, 1, 1.>(x, 1.);
}
template <typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

// Log10 for integers, rounding down
template <typename T>
    requires std::integral<T>
constexpr uint64_t IntegralLog10(const T val)
{
    // integer version can only ever return 0 as the lowest value
    T exp10 = 10;
    for(uint64_t i = 0; i < 20; i++)
    {
        if(val < exp10)
            return i;
        exp10 *= 10;
    }
    return 19; // max possible value for uint64_t
}

template <typename T>
    requires std::integral<T>
constexpr bool IsPowerOf10(const T val)
{
    // todo: consider val==0 a power of 10?
    if(val == 0)
        return true;

    uint64_t exp10 = 1;
    for(uint64_t i = 0; i < 20; i++)
    {
        if(val == exp10)
            return true;
        else if(val < exp10)
            return false;
        else // val > exp10
            exp10 *= 10;
    }
    return false;
}

//
// The following two functions are related to assigning set indices to each object in a list
// imagine the list of objects: {a, b, c, d}
// the set assignment {{a}, {b, c}, {d}} would translate to the indices: {0, 1, 1, 2}
// the set assignment {{a, c}, {b, d}} would translate to the indices: {0, 1, 0, 1}
// etc.
//
// The first value will _always_ be 0. The next value will either be 0 or 1. The one after that can be 0, 1, or 2.
// Effectively, we need to create a number where each index has another base.

constexpr std::array<uint8_t, 32> NumberOfPossibilities()
{
    std::array<uint8_t, 32> values{};
    for(std::size_t i = 0; i < values.size(); ++i)
        values.at(i) = (uint8_t)i + 1;
    return values;
}

constexpr std::array<uint64_t, 32> CumulativeSize()
{
    constexpr auto possibilities = NumberOfPossibilities();
    std::array<uint64_t, 32> values;
    values.at(0) = 1;
    for(std::size_t i = 1; i < values.size(); ++i)
        values.at(i) = values.at(i - 1) * possibilities.at(i);
    return values;
}

template <typename TReturn, std::ranges::range TRange>
const TReturn SetAssingmentsToInteger(const TRange& range)
{
    static constexpr auto possibilities = NumberOfPossibilities();
    static constexpr auto cumulativeSize = CumulativeSize();
    // todo
    return TReturn{};
}

template <std::ranges::range TRange, std::integral TInteger>
void IntegerToSetAssingment(const TInteger integer, TRange& range)
{
    // todo
}

// https://en.wikipedia.org/wiki/Stirling_numbers_of_the_second_kind

// NOTE: related to stirling number
// I need  \Sum from k=1 to n S(n,k)

// stirling number of the second kind
// n: number of elements
// k: number of sets
constexpr uint64_t Stirling(const uint64_t n, const uint64_t k)
{
    if(k == 1 || n == k)
        return 1;
    return (k * Stirling(n - 1, k)) + Stirling(n - 1, k - 1);
}

// generating permutations (Knuths algorithm):
// https://math.stackexchange.com/questions/222780/enumeration-of-set-partitions

template <uint32_t n, uint32_t k>
constexpr uint64_t NumberOfPermutationsHelper()
{
    if constexpr(n == k)
        return Stirling(n, k);
    else
        return Stirling(n, k) + NumberOfPermutationsHelper<n, k + 1>();
}

template <uint32_t n>
constexpr uint64_t NumberOfPermutations()
{
    return NumberOfPermutationsHelper<n, 1>();
}

//
template <uint32_t n, uint32_t k>
constexpr auto StirlingPermutations()
{
    constexpr auto arraySize = Stirling(n, k);

    using Permutation = std::array<uint8_t, n>;
    using Permutations = std::array<Permutation, arraySize>;

    return Permutations{};
}

template <uint32_t n>
using Partition = std::array<uint8_t, n>;

template <uint32_t n, uint32_t nPartitions>
using Partitions = std::array<Partition<n>, nPartitions>;

template <uint32_t n, uint32_t nPartitions, uint32_t depth>
constexpr void ConstructPartitionsRecursive(Partitions<n, nPartitions>& partitions, //
                                            Partition<n>& buffer,
                                            uint32_t& iPartition,
                                            const uint32_t nSubsets)
{
    if constexpr(depth == n)
    {
        partitions.at(iPartition) = buffer;
        iPartition++; // next Partition will be stored in the next index
    }
    else
    {
        for(uint32_t iSubset = 0; iSubset < nSubsets; ++iSubset)
        {
            buffer.at(depth) = iSubset;
            ConstructPartitionsRecursive<n, nPartitions, depth + 1>(partitions, buffer, iPartition, nSubsets);
        }

        buffer.at(depth) = nSubsets;
        ConstructPartitionsRecursive<n, nPartitions, depth + 1>(partitions, buffer, iPartition, nSubsets + 1);
    }
}

template <uint32_t n, uint32_t nPartitions>
constexpr auto ConstructPartitionsHelper()
{
    Partitions<n, nPartitions> partitions{};
    Partition<n> partition{};
    uint32_t iPartition = 0;

    ConstructPartitionsRecursive<n, nPartitions, 0>(partitions, partition, iPartition, 0);

    return partitions;
}

template <uint32_t n>
constexpr auto ConstructPartitions()
{
    constexpr uint64_t numberOfPartitions = NumberOfPermutations<n>();
    return ConstructPartitionsHelper<n, numberOfPartitions>();
}

//
//
//

// todo: make a general lookup wrapper?
template <typename T, std::size_t sizeI, std::size_t sizeJ>
struct GCDLookup
{
    static constexpr std::size_t sTableSizeI = sizeI;
    static constexpr std::size_t sTableSizeJ = sizeJ;

    using TableType = std::array<std::array<T, sizeJ>, sizeI>;
    //

    constexpr T GCD(const T first, const T second) const
    {
        // if we precomputed up to this point, return lookup
        if(first < sTableSizeI && second < sTableSizeJ)
            return sLookuptable[first][second];
        // else, compute
        return std::gcd(first, second);
    }

private:
    static constexpr TableType ConstructGCDLookupTable()
    {
        TableType table{};
        for(std::size_t i = 0; i < sTableSizeI; ++i)
            for(std::size_t j = 0; j < sTableSizeJ; ++j)
                table[i][j] = std::gcd(i, j);
        return table;
    }
    static constexpr TableType sLookuptable = ConstructGCDLookupTable();
};

template <typename TLambda>
struct LookupTable
{
public:
    // using InputType =
    // uisng ResultType = std::result_of
    constexpr LookupTable(TLambda lambda)
        : mLambda(lambda)
    {
        //
    }

private:
    TLambda mLambda;
};

} // namespace math
} // namespace vic