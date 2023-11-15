

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

} // namespace vic