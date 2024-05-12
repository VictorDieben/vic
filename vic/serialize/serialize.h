#pragma once

#include <algorithm>
#include <cstddef>
#include <expected>
#include <span>
#include <type_traits>

#include "vic/utils/as_tuple.h"
#include "vic/utils/concepts.h"
#include "vic/utils/variant.h"

namespace vic
{
namespace serialize
{

// todo: look at this repo:
// https://github.com/eyalz800/zpp_bits

// it contains some nice features, but is still limited by the functionality in c++20.
// mostly unpacking of pod-structs

using DefaultContainerSizeType = uint32_t; // i don't think a serialization should ever need more than 2^32 items

using DefaultVariantIndexType = uint16_t; // 65000 variants should be enough

template <typename T>
concept ConceptTrivial = std::is_trivial_v<T>;

template <typename T>
concept ConceptRange = requires(T obj) {
    obj.begin();
    obj.end();
};

template <typename T>
concept ConceptSet = ConceptRange<T> && requires(T obj) {
    // todo
    typename T::key_type;
    typename T::value_type;
    obj.insert(std::declval<typename T::key_type>());
};

template <typename T>
concept ConceptMap = ConceptRange<T> && requires(T obj) {
    typename T::key_type;
    typename T::mapped_type;
    typename T::value_type;
    obj.insert(std::declval<typename T::value_type>());
};

template <typename T>
concept ConceptExpected = requires(T obj) {
    typename T::value_type;
    typename T::error_type;
    typename T::unexpected_type;
    {
        obj.value()
    } -> std::convertible_to<typename T::value_type>;
    {
        obj.error()
    } -> std::convertible_to<typename T::error_type>;
};

template <typename T>
concept ConceptOptional = requires(T obj) {
    typename T::value_type;
    {
        obj.value()
    } -> std::convertible_to<typename T::value_type>;
    {
        obj.has_value()
    } -> std::convertible_to<bool>;
};

enum class StatusCode
{
    Ok,
    Error, // generic
    OutOfRange, // not enough bytes left over to perform conversion
    Unsupported // T is not yet supported for serialization
};

using SerializeStatus = std::expected<std::span<std::byte>, StatusCode>;
using DeserializeStatus = std::expected<std::span<const std::byte>, StatusCode>;

template <typename T>
concept ConceptDataTrivial = std::contiguous_iterator<typename T::iterator> && //
                             std::is_trivially_copyable_v<typename std::iterator_traits<typename T::iterator>::value_type> && //
                             requires(T& item) {
                                 item.begin();
                                 item.end();
                                 item.data();
                             };

// forward declare serialize
template <typename T>
constexpr SerializeStatus Serialize(std::span<std::byte>& buffer, const T& item);

template <typename T, typename... Ts>
constexpr SerializeStatus SerializeMany(std::span<std::byte>& buffer, const T& item, const Ts&... rest)
{
    auto res = Serialize(buffer, item);
    if(!res)
        return res; // error, return

    if constexpr(sizeof...(Ts) > 0)
        return SerializeMany(res.value(), rest...);
    else
        return res; // done
}

template <typename T>
    requires ConceptTrivial<T>
constexpr SerializeStatus SerializeTrivial(std::span<std::byte>& buffer, const T& item)
{
    if(buffer.size() < sizeof(T))
        return std::unexpected{StatusCode::OutOfRange};

    const std::byte* begin = reinterpret_cast<const std::byte*>(std::addressof(item));
    const std::byte* end = begin + sizeof(T);
    std::copy(begin, end, buffer.begin());

    return buffer.subspan(sizeof(T));
}

template <typename T>
    requires tuple_like<T>
constexpr SerializeStatus SerializeTupleLike(std::span<std::byte>& buffer, const T& item)
{
    auto f = [&](auto... xs) { return SerializeMany(buffer, xs...); };
    return std::apply(f, item);
}

template <typename T>
    requires ConceptAggregate<T>
constexpr SerializeStatus SerializeAgregate(std::span<std::byte>& buffer, const T& item)
{
    const auto tup = vic::as_tuple(item);
    return SerializeTupleLike(buffer, tup);
}

template <typename T>
    requires ConceptVariant<T>
constexpr SerializeStatus SerializeVariant(std::span<std::byte>& buffer, const T& item)
{
    // first encode which type is contained in the variant
    constexpr auto alternatives = std::variant_size_v<T>;
    const auto index = (DefaultVariantIndexType)item.index();
    auto res = Serialize(buffer, index);
    if(!res)
        return res;
    std::span<std::byte> newBuffer = res.value();

    // then serialize this type
    return std::visit(
        [&](auto& subItem) {
            return Serialize(newBuffer, subItem); //
        },
        item);
}

template <typename T>
    requires ConceptExpected<T>
constexpr SerializeStatus SerializeExpected(std::span<std::byte>& buffer, const T& item)
{
    const bool hasValue = item.has_value();
    auto res = Serialize(buffer, hasValue);
    if(!res)
        return res;
    std::span<std::byte> newBuffer = res.value();

    if(hasValue)
        return Serialize(newBuffer, item.value());
    else
        return Serialize(newBuffer, item.error());
}

template <typename T>
    requires ConceptOptional<T>
constexpr SerializeStatus SerializeOptional(std::span<std::byte>& buffer, const T& item)
{
    const bool hasValue = item.has_value();
    auto res = Serialize(buffer, hasValue);
    if(!res || !hasValue)
        return res;
    std::span<std::byte> newBuffer = res.value();

    return Serialize(newBuffer, item.value());
}

template <typename T>
    requires ConceptDataTrivial<T>
constexpr SerializeStatus SerializeDataTrivial(std::span<std::byte>& buffer, const T& item)
{
    // copy whole range at once
    using InnerType = typename std::iterator_traits<decltype(item.begin())>::value_type;
    const auto size = (DefaultContainerSizeType)std::distance(item.begin(), item.end());
    auto res = Serialize(buffer, size);
    if(!res || size == 0)
        return res;
    std::span<std::byte> newBuffer = res.value();
    const auto byteSize = size * sizeof(InnerType);
    if(newBuffer.size() < byteSize)
        return std::unexpected{StatusCode::OutOfRange};
    std::memcpy(&newBuffer.front(), item.data(), byteSize);
    return newBuffer.subspan(byteSize);
}

template <typename T>
    requires ConceptRange<T>
constexpr SerializeStatus SerializeRange(std::span<std::byte>& buffer, const T& item)
{
    using InnerType = typename std::iterator_traits<typename T::iterator>::value_type;

    if constexpr(ConceptDataTrivial<T>)
    {
        return SerializeDataTrivial(buffer, item);
    }
    else
    {
        const auto size = (DefaultContainerSizeType)std::distance(item.begin(), item.end());

        auto res = Serialize(buffer, size);
        if(!res || size == 0)
            return res;
        std::span<std::byte> loopBuffer = res.value();

        for(const auto& subItem : item)
            if(auto result = Serialize(loopBuffer, subItem))
                loopBuffer = result.value();
            else
                return result;
        return loopBuffer;
    }
}

template <typename T>
constexpr SerializeStatus Serialize(std::span<std::byte>& buffer, const T& item)
{
    // check that we support this data type
    static_assert(!ConceptPointer<T>);

    if constexpr(std::is_trivial_v<T>)
        return SerializeTrivial(buffer, item);

    else if constexpr(ConceptRange<T>)
        return SerializeRange(buffer, item);

    else if constexpr(ConceptVariant<T>)
        return SerializeVariant(buffer, item);

    else if constexpr(ConceptExpected<T>)
        return SerializeExpected(buffer, item);

    else if constexpr(ConceptOptional<T>)
        return SerializeOptional(buffer, item);

    else if constexpr(vic::tuple_like<T>)
        return SerializeTupleLike(buffer, item);

    else if constexpr(ConceptAggregate<T>)
        return SerializeAgregate(buffer, item);

    return std::unexpected{StatusCode::Unsupported};
}

//
//
//

// forward declare
template <typename T>
constexpr DeserializeStatus Deserialize(const std::span<const std::byte>& buffer, T& item);

template <typename T, typename... Ts>
constexpr DeserializeStatus DeserializeMany(const std::span<const std::byte>& buffer, T& item, Ts&... rest)
{
    const auto res = Deserialize(buffer, item);
    if(!res)
        return res;

    if constexpr(sizeof...(Ts) > 0)
        return DeserializeMany(res.value(), rest...);
    else
        return res;
}

template <typename T>
    requires ConceptTrivial<T>
constexpr DeserializeStatus DeserializeTrivial(const std::span<const std::byte>& buffer, T& item)
{
    if(buffer.size() < sizeof(T))
        return std::unexpected{StatusCode::OutOfRange};
    item = *reinterpret_cast<const T*>(&buffer.front());
    return buffer.subspan(sizeof(T));
}

template <typename T>
    requires tuple_like<T>
constexpr DeserializeStatus DeserializeTupleLike(const std::span<const std::byte>& buffer, T& item)
{
    auto f = [&](auto&... xs) { return DeserializeMany(buffer, xs...); };
    return std::apply(f, item);
}

template <typename T>
    requires ConceptAggregate<T>
constexpr DeserializeStatus DeserializeAgregate(const std::span<const std::byte>& buffer, T& item)
{
    const auto tup = vic::as_tuple(item);
    return DeserializeTupleLike(buffer, tup);
}

template <typename T>
    requires ConceptVariant<T>
constexpr DeserializeStatus DeserializeVariant(const std::span<const std::byte>& buffer, T& item)
{
    DefaultVariantIndexType idx;
    auto res = Deserialize(buffer, idx);
    if(!res)
        return res;
    if(idx >= std::variant_size_v<T>)
        return std::unexpected{StatusCode::Error}; // invalid type index

    AssignNthType(item, idx); // set variant to n-th index type

    return std::visit(
        [&](auto& subItem) -> DeserializeStatus {
            return Deserialize(res.value(), subItem); //
        },
        item);
}

template <typename T>
    requires ConceptExpected<T>
constexpr DeserializeStatus DeserializeExpected(const std::span<const std::byte>& buffer, T& item)
{
    bool hasValue;
    auto res = Deserialize(buffer, hasValue);
    if(!res)
        return res;
    std::span<const std::byte> newBuffer = res.value();

    if(hasValue)
    {
        item = typename T::value_type{};
        return Deserialize(newBuffer, item.value());
    }
    else
    {
        item = std::unexpected{typename T::error_type{}};
        return Deserialize(newBuffer, item.error());
    }
}

template <typename T>
    requires ConceptDataTrivial<T>
constexpr DeserializeStatus DeserializeDataTrivial(const std::span<const std::byte>& buffer, T& item)
{
    using InnerType = typename std::iterator_traits<typename T::iterator>::value_type;
    DefaultContainerSizeType s;
    auto res = Deserialize(buffer, s);
    if(!res || s == 0)
        return res;
    std::span<const std::byte> newBuffer = res.value();

    // entire data range can be copied at once
    const auto byteSize = s * sizeof(InnerType);
    if(newBuffer.size() < byteSize)
        return std::unexpected{StatusCode::OutOfRange};
    item.resize(s);
    std::memcpy(item.data(), &newBuffer.front(), byteSize);
    return newBuffer.subspan(byteSize);
}

template <typename T>
    requires ConceptRange<T>
constexpr DeserializeStatus DeserializeRange(const std::span<const std::byte>& buffer, T& item)
{
    using InnerType = typename std::iterator_traits<typename T::iterator>::value_type;
    DefaultContainerSizeType s;
    auto res = Deserialize(buffer, s);
    if(!res || s == 0)
        return res;
    std::span<const std::byte> newBuffer = res.value();

    if constexpr(ConceptDataTrivial<T>)
    {
        return DeserializeDataTrivial(buffer, item);
    }
    else if constexpr(ConceptMap<T>)
    {
        // map-like
        using KeyValue = std::pair<typename T::key_type, typename T::mapped_type>; // manually remove const-ness of key
        for(DefaultContainerSizeType i = 0; i < s; ++i)
        {
            KeyValue kv;
            if(res = Deserialize(res.value(), kv))
                item.insert(kv);
            else
                return res; // error
        }
        return res;
    }
    else if constexpr(ConceptSet<T>)
    {
        // set-like
        for(DefaultContainerSizeType i = 0; i < s; ++i)
        {
            typename T::value_type v;
            if(res = Deserialize(res.value(), v))
                item.insert(v);
            else
                return res; // error
        }
        return res;
    }
    else if constexpr(ConceptRange<T> && requires { item.resize({}); })
    {
        // items in list arn't trivial, or list is not contiguous
        item.resize(s);
        for(DefaultContainerSizeType i = 0; i < s; ++i)
        {
            if(res = Deserialize(res.value(), item.at(i)))
            {
                // success
            }
            else
                return res; // error
        }
        return res;
    }

    return std::unexpected{StatusCode::Unsupported};
}

template <typename T>
    requires ConceptOptional<T>
constexpr DeserializeStatus DeserializeOptional(const std::span<const std::byte>& buffer, T& item)
{
    bool hasValue;
    auto res = Deserialize(buffer, hasValue);
    if(!res || !hasValue)
        return res;
    std::span<const std::byte> newBuffer = res.value();

    item = typename T::value_type{}; // initialize
    return Deserialize(newBuffer, *item);
}

template <typename T>
constexpr DeserializeStatus Deserialize(const std::span<const std::byte>& buffer, T& item)
{
    if constexpr(ConceptTrivial<T>)
        return DeserializeTrivial(buffer, item);

    else if constexpr(ConceptRange<T>)
        return DeserializeRange(buffer, item);

    else if constexpr(ConceptVariant<T>)
        return DeserializeVariant(buffer, item);

    else if constexpr(ConceptExpected<T>)
        return DeserializeExpected(buffer, item);

    else if constexpr(ConceptOptional<T>)
        return DeserializeOptional(buffer, item);

    else if constexpr(vic::tuple_like<T>)
        return DeserializeTupleLike(buffer, item);

    else if constexpr(ConceptAggregate<T>)
        return DeserializeAgregate(buffer, item);

    return std::unexpected{StatusCode::Unsupported};
}

} // namespace serialize
} // namespace vic