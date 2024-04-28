#pragma once

#include <expected>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vic/utils/error.h"

namespace vic
{

enum class ReadFileErrorCode
{
    DoesNotExists,
    InvalidPath,
    NotRegularFile,
    FailedToOpen,
    FailedToRead,
    Other
};

using FileData = std::vector<char>;
using ReadFileError = ::vic::StaticError<ReadFileErrorCode>;
using ReadFileResult = std::expected<FileData, ReadFileError>;

inline ReadFileResult FileToCharVec(const std::filesystem::path& filePath)
{
    if(!std::filesystem::is_regular_file(filePath))
        return std::unexpected{ReadFileError{ReadFileErrorCode::NotRegularFile, //
                                             std::format("file \'{}\' is not a regular file!", filePath.generic_string())}};

    std::ifstream file{filePath, std::ios::ate | std::ios::binary};
    if(!file.is_open())
        return std::unexpected{ReadFileError{ReadFileErrorCode::FailedToOpen, //
                                             std::format("failed to open file: {}", filePath.generic_string())}};

    try
    {
        const auto fileSize = static_cast<std::size_t>(file.tellg());
        std::vector<char> result(fileSize);
        file.seekg(0);
        file.read(result.data(), fileSize);
        return result; // no need to call close, done by ifstream destructor
    }
    catch(...)
    {
        return std::unexpected{ReadFileError{ReadFileErrorCode::FailedToRead, //
                                             "failed to read file: " + filePath.generic_string()}};
    }
}

enum class SaveFileErrorCode
{
    Other
};

using SaveFileError = ::vic::StaticError<SaveFileErrorCode>;
using SaveFileResult = std::expected<void, SaveFileError>;

inline SaveFileResult CharVecToFile(const std::filesystem::path& path, //
                                    const FileData& data)
{
    try
    {
        std::ofstream output(path, std::ios::out | std::ios::binary);
        output.write((const char*)&data[0], data.size());
    }
    catch(...)
    {
        return std::unexpected{SaveFileError{SaveFileErrorCode::Other, //
                                             "failed to save file: " + path.generic_string()}};
    }
}

} // namespace vic