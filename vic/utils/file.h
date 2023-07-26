#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <filesystem>

namespace vic
{

inline std::vector<char> FileToCharVec(const std::filesystem::path& filePath)
{
    if(!std::filesystem::is_regular_file(filePath))
        throw std::runtime_error("file: \'" + filePath.generic_string() + "\' is not a regular file!");

    std::ifstream file{filePath, std::ios::ate | std::ios::binary};
    if(!file.is_open())
        throw std::runtime_error("failed to open file: " + filePath.generic_string());

    const auto fileSize = static_cast<std::size_t>(file.tellg());
    std::vector<char> result(fileSize);

    file.seekg(0);
    file.read(result.data(), fileSize);
    return result; // no need to call close, done by ifstream destructor
}

} // namespace vic