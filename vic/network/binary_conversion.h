#pragma once

#include <array>
#include <type_traits>

namespace vic
{
namespace network
{

using Byte = std::uint8_t;

// base template
template <typename T>
struct BinaryConversion;

// fundamental type template
template <typename T>
struct BinaryConversion
{
    static_assert(std::is_fundamental_v<T>);

public:
    constexpr std::size_t ByteSize() const { return sizeof(T); }
    static std::size_t ByteSize(const T& object)
    {
        (void)object;
        return ByteSize();
    }

    template <typename TIter>
    static void Read (T& object, TIter begin, TIter end){};

    template <typename TIter>
    static void Write (const T& object, TIter begin, TIter end){};
};

// implementation for std::array<T, N>

template <class T>
struct is_std_array : std::false_type
{ };

template <class T, size_t N>
struct is_std_array<std::array<T, N>> : std::true_type
{ };

template <class T>
constexpr bool is_std_array_v = is_std_array<std::remove_cvref_t<T>>::value;

//template <typename T>
//struct BinaryConversion
//{
//    static_assert(std::is_std_array_v<T>);
//
//public:
//    constexpr std::size_t ByteSize() const { return sizeof(T); }
//    static std::size_t ByteSize(const T& object)
//    {
//        (void)object;
//        return ByteSize();
//    }
//
//    template <typename TIter>
//    static void Read (T& object, TIter begin, TIter end){}
//
//    template <typename TIter>
//    static void Write (const T& object, TIter begin, TIter end){}
//};

}

} // namespace vic