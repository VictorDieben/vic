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
    std::vector<std::byte> buffer;
    buffer.reserve(1024); // one KiB

    const auto res = vic::serialize::Serialize(std::back_insert_iterator(buffer), item);
    if(!res)
        return false;

    const auto trimmed = std::span<const std::byte>{buffer.begin(), buffer.size()};

    T newItem;
    if(!vic::serialize::Deserialize(trimmed, newItem))
        return false;
    return item == newItem;
};