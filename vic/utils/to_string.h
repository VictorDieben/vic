#pragma once

#include <array>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <iostream>
#include <string>

namespace std
{

template <typename T>
ostream& operator<<(ostream& os, const vector<T>& vec)
{
    os << "[";
    for(auto it = vec.begin(); it != vec.end(); ++it)
    {
        os << *it;
        if(it != prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T, std::size_t N>
ostream& operator<<(ostream& os, const array<T, N>& vec)
{
    os << "[";
    for(auto it = vec.cbegin(); it != vec.cend(); ++it)
    {
        os << *it;
        if(it != prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T>
ostream& operator<<(ostream& os, const set<T>& vec)
{
    os << "[";
    for(auto it = vec.cbegin(); it != vec.cend(); ++it)
    {
        os << *it;
        if(it != prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T>
ostream& operator<<(ostream& os, const unordered_set<T>& vec)
{
    os << "[";
    for(const auto it = vec.begin(); it != vec.end(); ++it)
    {
        os << *it;
        if(it != prev(vec.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

template <typename T1, typename T2>
ostream& operator<<(ostream& os, const map<T1, T2>& map)
{
    os << "[";
    for(auto it = map.begin(); it != map.end(); ++it)
    {
        os << "(" << it->first << "," << it->second << ")";
        if(it != prev(map.end()))
            os << ", ";
    }
    os << "]";
    return os;
}

} // namespace std

namespace vic
{
template <typename T>
std::string ToString(const T& item)
{
    std::ostringstream stream;
    stream << item;
    return stream.str();
}

} // namespace vic