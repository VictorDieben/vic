#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace vic
{
template <typename T>
std::vector<T> FileToVec(const std::string& filePath)
{
    std::ifstream file{filePath, std::ios::ate | std::ios::binary};
    if(!file.is_open())
        throw std::runtime_error("failed to open file: " + filePath);

    const auto fileSize = static_cast<std::size_t>(file.tellg());
    std::vector<T> result(fileSize);

    file.seekg(0);
    file.read(result.data(), fileSize);
    return result; // no need to call close, done by ifstream destructor
}

} // namespace vic