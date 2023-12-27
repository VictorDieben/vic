#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace vic
{

void Split(std::vector<std::string_view>& buffer, //
           const std::string_view& str,
           const std::string_view& delimiter)
{
    buffer.clear();

    typename std::size_t index = 0;
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

std::vector<std::string_view> Split(const std::string_view& str, //
                                    const std::string_view& delimiter)
{
    std::vector<std::string_view> buffer;
    Split(buffer, str, delimiter);
    return buffer;
}

} // namespace vic