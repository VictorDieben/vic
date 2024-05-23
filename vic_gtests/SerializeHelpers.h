#pragma once

#include <concepts>
#include <expected>
#include <string>
#include <variant>
#include <vector>

#include "vic/utils/variant.h"
#include <vic/utils/error.h>

// define some types for serialization/deserialization

enum class MyEnum : int
{
    First,
    Second
};

struct MyStruct
{
    std::string name;
    int age;
};
bool operator==(const MyStruct& a, const MyStruct& b) { return (a.name == b.name) && (a.age == b.age); }

struct MyOtherStruct
{
    int a;
    double b;
};
bool operator==(const MyOtherStruct& a, const MyOtherStruct& b) { return (a.a == b.a) && (a.b == b.b); }

struct MyNestedStruct
{
    MyStruct m1;
    MyOtherStruct m2;
};

// roundtrip serialization and deserialization.
template <typename T>
    requires std::equality_comparable<T>
T SerializeDeserialize(const T& item)
{
    std::vector<std::byte> buffer;
    buffer.reserve(1024); // one KiB

    const auto res = vic::serialize::Serialize(std::back_insert_iterator(buffer), item);

    T newItem{};
    if(res)
    {
        const auto trimmed = std::span<const std::byte>{buffer.begin(), buffer.size()};
        vic::serialize::Deserialize(trimmed, newItem);
    }
    return newItem;
};

template <typename T>
    requires std::equality_comparable<T>
bool RoundtripEquals(const T& item)
{
    const auto roundtrip = SerializeDeserialize(item);
    return item == roundtrip;
}

namespace chat
{
using RpcId = uint64_t;

enum class ErrorTypeEnum
{
    NameUnavailable,
    General
};
using ErrorType = vic::StaticError<ErrorTypeEnum>;

struct ChatConnect
{
    std::string name;
};

inline bool operator==(const ChatConnect& a, const ChatConnect& b) { return a.name == b.name; }

struct ChatConnectReply
{
    struct Value
    {
        std::string name;
        int signinTime;
        bool operator==(const Value& other) const { return (name == other.name) && (signinTime == other.signinTime); }
    };

    std::expected<Value, ErrorType> reply;
};
inline bool operator==(const ChatConnectReply& a, const ChatConnectReply& b) { return a.reply == b.reply; }

struct ChatSendMessage
{
    std::string message;
};
inline bool operator==(const ChatSendMessage& a, const ChatSendMessage& b) { return a.message == b.message; }

struct ChatSendMessageReply
{
    bool succes;
    std::expected<bool, ErrorType> reply;
};

inline bool operator==(const ChatSendMessageReply& a, const ChatSendMessageReply& b) { return a.succes == b.succes; }

struct ChatStreamMessage
{
    std::string message;
};

inline bool operator==(const ChatStreamMessage& a, const ChatStreamMessage& b) { return a.message == b.message; }

using ChatVariant = std::variant<ChatConnect, // client -> server
                                 ChatSendMessage,
                                 ChatConnectReply, // server -> client
                                 ChatSendMessageReply,
                                 ChatStreamMessage>;

using ChatRpc = std::pair<RpcId, ChatVariant>;

inline bool operator==(const ChatRpc& a, const ChatRpc& b) { return (a.first == b.first) && (a.second == b.second); }

} // namespace chat