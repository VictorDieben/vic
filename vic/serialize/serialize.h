#pragma once

#include <algorithm>
#include <cstddef>
#include <expected>
#include <span>
#include <type_traits>

#include "vic/utils/as_tuple.h"
#include "vic/utils/concepts.h"

namespace vic
{
namespace serialize
{

// todo: look at this repo:
// https://github.com/eyalz800/zpp_bits

// it contains some nice features, but is still limited by the functionality in c++20.
// mostly unpacking of pod-structs

using DefaultContainerSizeType = uint32_t; // i don't think a serialization should ever need more than 2^32 items

template <typename T>
concept ConceptSerializer = requires(T obj) {
    obj.serialize();
    obj.deserialize();
};

template <typename T>
constexpr uint64_t ByteSize()
{
    return sizeof(T); // todo: for vectors / strings etc we need to return the size of the actual serialization
}

template <typename T>
uint64_t ByteSize(const T& item)
{
    // todo: for vectors / strings etc we need to return the size of the actual serialization
    return sizeof(T);
}

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
struct Serializer
{
    Serializer(std::span<std::byte>& buffer)
        : mBuffer(buffer)
    { }

    static constexpr bool FixedSize = true;
    // static constexpr std::uint64_t size =

    SerializeStatus Serialize()
    {
        //
        return SerializeStatus{};
    }

    SerializeStatus Deserialize()
    {
        //
        return SerializeStatus{};
    }

private:
    std::span<std::byte>& mBuffer;
};

// forward declare serialize
template <typename T, typename... Ts>
constexpr SerializeStatus Serialize(std::span<std::byte>& buffer, const T& item, const Ts&... rest);

template <typename T>
constexpr SerializeStatus SerializeOne(std::span<std::byte>& buffer, const T& item)
{
    // check that we support this data type
    static_assert(!ConceptPointer<T>);

    if constexpr(std::is_trivial_v<T>) // todo: or something like is_trivial<T>
    {
        if(buffer.size() < sizeof(T))
            return std::unexpected{StatusCode::OutOfRange};

        // todo: find out why std::to_bytes is not working
        const std::byte* begin = reinterpret_cast<const std::byte*>(std::addressof(item));
        const std::byte* end = begin + sizeof(T);
        std::copy(begin, end, buffer.begin());

        return buffer.subspan(sizeof(T));
    }
    else if constexpr(requires {
                          item.begin();
                          item.end();
                      })
    {
        using InnerType = typename std::iterator_traits<decltype(item.begin())>::value_type;

        const auto size = (DefaultContainerSizeType)std::distance(item.begin(), item.end());
        auto serializeSize = Serialize(buffer, size);
        if(!serializeSize)
            return serializeSize;

        std::span<std::byte> loopBuffer = serializeSize.value();

        if constexpr(requires {
                         std::contiguous_iterator<decltype(item.begin())>;
                         std::is_trivial<InnerType>;
                     })
        {
            // copy whole range at once
            const auto byteSize = size * sizeof(InnerType);
            if(loopBuffer.size() < byteSize)
                return std::unexpected{StatusCode::OutOfRange};
            std::memcpy(&loopBuffer.front(), &item.at(0), byteSize);
            return buffer.subspan(byteSize);
        }
        else
        {
            // copy each individually
            for(const auto& subItem : item)
                if(auto result = Serialize(loopBuffer, subItem))
                    loopBuffer = result.value();
                else
                    return result;

            return loopBuffer;
        }
    }
    else if constexpr(ConceptAggregate<T>)
    {
        const auto tup = vic::as_tuple(item);

        // make a wrapper lambda, to pre-apply buffer as an argument
        auto f = [&](auto... xs) { return Serialize(buffer, xs...); };
        auto res = std::apply(f, tup);

        return res;
    }

    return std::unexpected{StatusCode::Unsupported};
}

template <typename T, typename... Ts>
constexpr SerializeStatus Serialize(std::span<std::byte>& buffer, const T& item, const Ts&... rest)
{
    auto res = SerializeOne(buffer, item);
    if(!res)
        return res; // error, return

    if constexpr(sizeof...(Ts) > 0)
        return Serialize(res.value(), rest...);
    else
        return res; // done
}

template <typename T, typename... Ts>
constexpr DeserializeStatus Deserialize(const std::span<const std::byte>& buffer, T& item, Ts&... rest);

template <typename T>
constexpr DeserializeStatus DeserializeOne(const std::span<const std::byte>& buffer, T& item)
{
    if constexpr(std::is_trivial_v<T>)
    {
        if(buffer.size() < sizeof(T))
            return std::unexpected{StatusCode::OutOfRange};
        item = *reinterpret_cast<const T*>(&buffer.front());
        return buffer.subspan(sizeof(T));
    }
    else if constexpr(requires {
                          item.begin();
                          item.end();
                      })
    {
        using InnerType = typename std::iterator_traits<decltype(item.begin())>::value_type;
        DefaultContainerSizeType s;
        auto res = Deserialize(buffer, s);
        if(!res || s == 0)
            return res;
        std::span<const std::byte> newBuffer = res.value();

        if constexpr(requires {
                         std::contiguous_iterator<decltype(item.begin())>;
                         std::is_trivial<InnerType>;
                     })
        {
            // entire data range can be copied at once
            const auto byteSize = s * sizeof(InnerType);
            if(newBuffer.size() < byteSize)
                return std::unexpected{StatusCode::OutOfRange};
            std::memcpy(&item.at(0), &newBuffer.front(), byteSize);
            return buffer.subspan(byteSize);
        }
        else
        {
            if constexpr(requires {
                             item.resize(0);
                             item.at(0);
                         })
            {
                // items in list arn't trivial, or list is not contiguous
                item.resize(s);
                for(DefaultContainerSizeType i = 0; i < s; ++i)
                {
                    if(auto res = Deserialize(newBuffer, item.at(i)))
                        newBuffer = res.value();
                    else
                        return res;
                }
                return newBuffer;
            }
            else if constexpr(requires {
                                  typename T::key_type;
                                  typename T::mapped_type;
                                  typename T::value_type;
                                  item.insert(std::declval<typename T::value_type>());
                              })
            {
                // map-like
            }
            else if constexpr(requires {
                                  typename T::key_type;
                                  typename T::value_type;
                                  item.insert(std::declval<typename T::key_type>());
                              })
            {
                // set-like
            }
        }
    }
    else if constexpr(ConceptAggregate<T>)
    {
        const auto tup = vic::as_tuple(item);

        auto f = [&](auto&... xs) { return Deserialize(buffer, xs...); };
        auto res = std::apply(f, tup);

        return res;
    }

    return std::unexpected{StatusCode::Unsupported};
}

template <typename T, typename... Ts>
constexpr DeserializeStatus Deserialize(const std::span<const std::byte>& buffer, T& item, Ts&... rest)
{
    const auto res = DeserializeOne(buffer, item);
    if(!res)
        return res;

    if constexpr(sizeof...(Ts) > 0)
        return Deserialize(res.value(), rest...);
    else
        return res;
}

} // namespace serialize
} // namespace vic