#pragma once

#include <variant>

namespace vic
{
// helpers for std::variant

template <class... Ts>
struct overload : Ts...
{
    using Ts::operator()...;
};

template <typename T>
concept ConceptVariant = requires(T& item) {
    {
        item.index()
    } -> std::integral;
};

template <std::size_t N, typename TVariant, typename... Args>
    requires ConceptVariant<TVariant>
void AssignNthTypeHelper(TVariant& variant, const std::size_t index, Args&&... args)
{
    if constexpr(N >= std::variant_size_v<TVariant>)
        return; // throw?

    if(N == index)
    {
        using NthType = std::variant_alternative_t<N, TVariant>;
        if constexpr(requires { NthType(std::forward<Args>(args)...); }) // if T can be constructed from args
            variant.emplace<NthType>(std::forward<Args>(args)...);
        return;
    }

    if constexpr(N + 1 < std::variant_size_v<TVariant>)
        return AssignNthTypeHelper<N + 1>(variant, index, std::forward<Args>(args)...);
}

template <typename TVariant, typename... Args>
    requires ConceptVariant<TVariant>
void AssignNthType(TVariant& variant, const std::size_t index, Args&&... args)
{
    return AssignNthTypeHelper<0>(variant, index, std::forward<Args>(args)...);
}

} // namespace vic