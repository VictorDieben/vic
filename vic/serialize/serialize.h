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
concept ConceptResizeable = requires(T obj) {
    obj.resize({}); //
};

template <typename T>
concept ConceptInsertable = requires(T obj) {
    obj.insert({}); //
};

template <typename T>
concept ConceptResizeableRange = ConceptRange<T> && (ConceptResizeable<T> || ConceptInsertable<T>);

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
    Unsupported // T is not (yet?) supported for serialization
};

//struct SerializeRange
//{
//
//    using iterator = typename std::vector<std::byte>::iterator; // todo: make this work with arrays too
//
//    iterator begin;
//    iterator head;
//    iterator end;
//
//    //template <typename TContainer>
//    // SerializeRange
//};

template <typename TInsertionIterator>
    requires std::output_iterator<std::remove_reference_t<TInsertionIterator>, std::byte>
using SerializeStatus = std::expected<TInsertionIterator, StatusCode>;

using DeserializeStatus = std::expected<std::span<const std::byte>, StatusCode>;

template <typename T>
concept ConceptDataTrivial = std::contiguous_iterator<typename T::iterator> && //
                             std::is_trivially_copyable_v<typename std::iterator_traits<typename T::iterator>::value_type> && //
                             requires(T& item) {
                                 item.begin();
                                 item.end();
                                 item.data();
                                 item.size();
                             };

// forward declare serialize
template <typename T, typename TIter>
constexpr SerializeStatus<TIter> Serialize(TIter insertionIterator, const T& item);

template <typename T, typename TIter, typename... Ts>
constexpr SerializeStatus<TIter> SerializeMany(TIter insertionIterator, const T& item, const Ts&... rest)
{
    auto res = Serialize(insertionIterator, item);
    if(!res)
        return res; // error, return

    if constexpr(sizeof...(Ts) > 0)
        return SerializeMany(res.value(), rest...);
    else
        return res; // done
}

template <typename T, typename TIter>
    requires ConceptTrivial<T>
constexpr SerializeStatus<TIter> SerializeTrivial(TIter insertionIterator, const T& item)
{
    // todo: to span of bytes instead?
    const std::byte* begin = reinterpret_cast<const std::byte*>(std::addressof(item));
    const std::byte* end = begin + sizeof(T);
    return std::copy(begin, end, insertionIterator);
}

template <typename T, typename TIter>
    requires tuple_like<T>
constexpr SerializeStatus<TIter> SerializeTupleLike(TIter insertionIterator, const T& item)
{
    auto f = [&](auto... xs) { return SerializeMany(insertionIterator, xs...); };
    return std::apply(f, item);
}

template <typename T, typename TIter>
    requires ConceptAggregate<T>
constexpr SerializeStatus<TIter> SerializeAgregate(TIter insertionIterator, const T& item)
{
    const auto tup = vic::as_tuple(item);
    return SerializeTupleLike(insertionIterator, tup);
}

template <typename T, typename TIter>
    requires ConceptVariant<T>
constexpr SerializeStatus<TIter> SerializeVariant(TIter insertionIterator, const T& item)
{
    // first encode which type is contained in the variant
    constexpr auto alternatives = std::variant_size_v<T>;
    const auto index = (DefaultVariantIndexType)item.index();
    auto res = Serialize(insertionIterator, index);
    if(!res)
        return res;

    // then serialize this type
    return std::visit(
        [&](auto& subItem) {
            return Serialize(res.value(), subItem); //
        },
        item);
}

template <typename T, typename TIter>
    requires ConceptExpected<T>
constexpr SerializeStatus<TIter> SerializeExpected(TIter insertionIterator, const T& item)
{
    const bool hasValue = item.has_value();
    auto res = Serialize(insertionIterator, hasValue);
    if(!res)
        return res; // error

    if(hasValue)
        return Serialize(res.value(), item.value());
    else
        return Serialize(res.value(), item.error());
}

template <typename T, typename TIter>
    requires ConceptOptional<T>
constexpr SerializeStatus<TIter> SerializeOptional(TIter insertionIterator, const T& item)
{
    const bool hasValue = item.has_value();
    auto res = Serialize(insertionIterator, hasValue);
    if(!res || !hasValue)
        return res;

    return Serialize(res.value(), item.value());
}

template <typename T, typename TIter>
    requires ConceptDataTrivial<T>
constexpr SerializeStatus<TIter> SerializeDataTrivial(TIter insertionIterator, const T& item)
{
    // copy whole range at once
    const auto size = (DefaultContainerSizeType)item.size();
    auto res = Serialize(insertionIterator, size);
    if(!res || size == 0)
        return res;

    using ValueType = typename std::iterator_traits<typename T::iterator>::value_type;

    const std::byte* begin = reinterpret_cast<const std::byte*>(item.data());
    const std::byte* end = begin + (size * sizeof(ValueType));
    return std::copy(begin, end, res.value());
}

template <typename T, typename TIter>
    requires ConceptRange<T>
constexpr SerializeStatus<TIter> SerializeFixedRange(TIter insertionIterator, const T& item)
{
    SerializeStatus<TIter> res = insertionIterator;
    for(const auto& subItem : item)
        if(res = Serialize(res.value(), subItem); !res)
            return res; // error, do not continue
    return res;
}

template <typename T, typename TIter>
    requires ConceptRange<T>
constexpr SerializeStatus<TIter> SerializeRange(TIter insertionIterator, const T& item)
{
    using InnerType = typename std::iterator_traits<typename T::iterator>::value_type;

    if constexpr(ConceptDataTrivial<T>)
        return SerializeDataTrivial(insertionIterator, item);

    if constexpr(!ConceptResizeableRange<T>)
        return SerializeFixedRange(insertionIterator, item);

    const auto size = (DefaultContainerSizeType)item.size();

    auto res = Serialize(insertionIterator, size);
    if(!res || size == 0)
        return res;

    for(const auto& subItem : item)
        if(res = Serialize(res.value(), subItem); !res)
            return res;
    return res;
}

template <typename T, typename TIter>
constexpr SerializeStatus<TIter> Serialize(TIter insertionIterator, const T& item)
{
    // check that we support this data type
    static_assert(!ConceptPointer<T>);

    if constexpr(std::is_trivial_v<T>)
        return SerializeTrivial(insertionIterator, item);

    else if constexpr(ConceptRange<T>)
        return SerializeRange(insertionIterator, item);

    else if constexpr(ConceptVariant<T>)
        return SerializeVariant(insertionIterator, item);

    else if constexpr(ConceptExpected<T>)
        return SerializeExpected(insertionIterator, item);

    else if constexpr(ConceptOptional<T>)
        return SerializeOptional(insertionIterator, item);

    else if constexpr(vic::tuple_like<T>)
        return SerializeTupleLike(insertionIterator, item);

    else if constexpr(ConceptAggregate<T>)
        return SerializeAgregate(insertionIterator, item);

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
    if constexpr(requires { item.resize({s}); })
        item.resize(s);
    else if(item.size() != s)
        return std::unexpected{StatusCode::Error};
    std::memcpy(item.data(), &newBuffer.front(), byteSize);
    return newBuffer.subspan(byteSize);
}

template <typename T>
    requires ConceptMap<T>
constexpr DeserializeStatus DeserializeMap(const std::span<const std::byte>& buffer, T& item)
{
    using KeyValue = std::pair<typename T::key_type, typename T::mapped_type>; // manually remove const-ness from key
    DefaultContainerSizeType s;
    auto res = Deserialize(buffer, s);
    if(!res || s == 0)
        return res;

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

template <typename T>
    requires ConceptSet<T>
constexpr DeserializeStatus DeserializeSet(const std::span<const std::byte>& buffer, T& item)
{
    DefaultContainerSizeType s;
    auto res = Deserialize(buffer, s);
    if(!res || s == 0)
        return res;

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

template <typename T>
    requires ConceptRange<T>
constexpr DeserializeStatus DeserializeFixedSize(const std::span<const std::byte>& buffer, T& item)
{
    if constexpr(ConceptTrivial<T>)
        return DeserializeTrivial(buffer, item);

    DeserializeStatus res = buffer;
    for(DefaultContainerSizeType i = 0; i < item.size(); ++i)
        if(res = Deserialize(res.value(), item[i]); !res)
            return res; // error

    return res;
}

template <typename T>
    requires ConceptRange<T>
constexpr DeserializeStatus DeserializeRange(const std::span<const std::byte>& buffer, T& item)
{
    if constexpr(!ConceptResizeableRange<T>)
        return DeserializeFixedSize(buffer, item);

    else if constexpr(ConceptDataTrivial<T>)
        return DeserializeDataTrivial(buffer, item);

    else if constexpr(ConceptMap<T>)
        return DeserializeMap(buffer, item);

    else if constexpr(ConceptSet<T>)
        return DeserializeSet(buffer, item);

    else if constexpr(ConceptResizeableRange<T>)
    {
        using InnerType = typename std::iterator_traits<typename T::iterator>::value_type;
        DefaultContainerSizeType s;
        auto res = Deserialize(buffer, s);
        if(!res || s == 0)
            return res;
        std::span<const std::byte> newBuffer = res.value();

        // items in list arn't trivial, or list is not contiguous
        item.resize(s);
        for(DefaultContainerSizeType i = 0; i < s; ++i)
            if(res = Deserialize(res.value(), item.at(i)); !res)
                return res; // error
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

    else if constexpr(vic::tuple_like<T>) // should probably be checked before ConceptAgregate, and after ConceptTrivial
        return DeserializeTupleLike(buffer, item);

    else if constexpr(ConceptAggregate<T>)
        return DeserializeAgregate(buffer, item);

    return std::unexpected{StatusCode::Unsupported};
}

} // namespace serialize
} // namespace vic