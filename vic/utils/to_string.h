#pragma once

#include <array>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <iostream>
#include <string>

namespace vic
{

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
{
    os << "[";
    for(auto it = vec.begin(); it < vec.end(); ++it)
    {
        os << *it;
        if(it < std::prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<T, N>& vec)
{
    os << "[";
    for(auto it = vec.begin(); it < vec.end(); ++it)
    {
        os << *it;
        if(it < std::prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::set<T>& vec)
{
    os << "[";
    for(auto it = vec.begin(); it < vec.end(); ++it)
    {
        os << *it;
        if(it < std::prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::unordered_set<T>& vec)
{
    os << "[";
    for(auto it = vec.begin(); it < vec.end(); ++it)
    {
        os << *it;
        if(it < std::prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T>
std::string ToString(const T& item)
{
    std::ostringstream stream;
    stream << item;
    return stream.str();
}

} // namespace vic