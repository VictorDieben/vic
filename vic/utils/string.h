#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace vic
{

inline void Split(std::vector<std::string_view>& buffer, //
                  const std::string_view& str,
                  const std::string_view& delimiter)
{
    buffer.clear();

    std::size_t index = 0;
    while(index < str.size())
    {
        const std::size_t nextIndex = str.find(delimiter, index);
        if(index == nextIndex)
        {
            index += 1;
            continue;
        }

        if(nextIndex == std::string::npos)
        {
            buffer.push_back(std::string_view(str.begin() + index, str.begin() + str.size()));
            return;
        }

        buffer.push_back(std::string_view(str.begin() + index, str.begin() + nextIndex));
        index = nextIndex + 1;
    }
}

inline std::vector<std::string_view> Split(const std::string_view str, //
                                           const std::string_view delimiter)
{
    std::vector<std::string_view> buffer;
    Split(buffer, str, delimiter);
    return buffer;
}

template <typename T>
T ToIntegral(const std::string_view sv)
{
    // todo: really inefficient, there should just be a standard way of doing this
    auto tostring = std::string{sv};

    if constexpr(std::is_same_v<T, int>)
        return std::stoi(tostring);
    if constexpr(std::is_same_v<T, long>)
        return std::stol(tostring);
    if constexpr(std::is_same_v<T, long long>)
        return std::stoll(tostring);
    return T{}; // todo: this should be a compile error
}

} // namespace vic