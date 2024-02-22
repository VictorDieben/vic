

#pragma once

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

} // namespace vic