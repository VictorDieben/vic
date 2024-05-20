#pragma once

#include <string>
#include <vector>

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

// roundtrip serialization and deserialization. Make sure the reconstructed data is the same as the input
template <typename T>
bool SerializeDeserialize(const T& item)
{
    std::vector<std::byte> buffer(1024); // 1 KiB
    auto span = std::span<std::byte>{buffer};
    const auto res = vic::serialize::Serialize(span, item);
    if(!res)
        return false;

    // todo: this is a hack. we should not be comparing pointer values to determine the size.
    // maybe rewrite the serializer so that it uses iterators, then it would be valid.
    std::byte* front = &(*span.begin());
    std::byte* head = &(*res.value().begin());
    const auto trimmed = std::span<const std::byte>{front, (std::size_t)std::distance(front, head)};

    T newItem;
    if(!vic::serialize::Deserialize(trimmed, newItem))
        return false;
    return item == newItem;
};