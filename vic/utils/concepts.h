#pragma once

#include <cstddef>
#include <iterator>

namespace vic
{
// todo: replace with std concepts once available

template <typename T>
concept less_than_comparable = requires(T obj) {
    {
        obj < obj
    } -> std::same_as<bool>;
};

template <typename T>
concept equality_comparable = requires(T obj) {
    {
        obj == obj
    } -> std::same_as<bool>;
    {
        obj != obj
    } -> std::same_as<bool>;
};

template <typename T>
concept ConceptMap = requires(T map) {
    typename T::key_type;
    typename T::value_type;

    //{
    //    map.at(typename T::key_type{});
    //} -> std::same_as<T::value_type>;
};

template <typename T>
concept ConceptTuple = requires(T a) {
    std::tuple_size<T>::value;
    std::get<0>(a);
};

template <typename TMap1, typename TMap2>
concept ConceptSameKey = ConceptMap<TMap1> && ConceptMap<TMap2> && requires(TMap1& map1, TMap2& map2) {
    std::is_same_v<typename TMap1::key_type, typename TMap2::key_type>; //
};

template <class T>
concept Numeric = std::is_arithmetic_v<std::decay_t<T>>; // todo: replace with arithmetic type once in standard

template <class T>
concept ConceptFundamental = std::is_fundamental_v<std::decay_t<T>>; // todo: replace with arithmetic type once in standard

template <class T>
concept ConceptPointer = std::is_pointer_v<T>;

template <class T>
concept ConceptReference = std::is_reference_v<T>;

template <class T>
concept ConceptAggregate = std::is_aggregate_v<T>;

// the following is an implementation of the "exposition-only" definition of tuple_like, see:
// https://en.cppreference.com/w/cpp/utility/tuple/tuple-like

template <class T>
inline constexpr bool is_tuple_like_v = false;

template <class... Elems>
inline constexpr bool is_tuple_like_v<std::tuple<Elems...>> = true;

template <class T1, class T2>
inline constexpr bool is_tuple_like_v<std::pair<T1, T2>> = true;

template <class T, size_t N>
inline constexpr bool is_tuple_like_v<std::array<T, N>> = true;

template <class It, class Sent, std::ranges::subrange_kind Kind>
inline constexpr bool is_tuple_like_v<std::ranges::subrange<It, Sent, Kind>> = true;

template <class T>
concept tuple_like = is_tuple_like_v<std::remove_cvref_t<T>>;

template <typename T>
concept pair_like = tuple_like<T> && std::tuple_size_v<std::remove_cvref_t<T>> == 2;

} // namespace vic