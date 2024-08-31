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

template <typename T>
using FileData = std::vector<T>;

using ReadFileError = ::vic::StaticError<ReadFileErrorCode>;

template <typename T>
using ReadFileResult = std::expected<FileData<T>, ReadFileError>;

template <typename T>
// requires sizeof(T) == 1
ReadFileResult<T> FileToVec(const std::filesystem::path& path)
{
    if(!std::filesystem::is_regular_file(path))
        return std::unexpected{ReadFileError{ReadFileErrorCode::NotRegularFile, //
                                             std::format("file \'{}\' is not a regular file!", path.generic_string())}};

    std::ifstream file{path, std::ios::ate | std::ios::binary};
    if(!file.is_open())
        return std::unexpected{ReadFileError{ReadFileErrorCode::FailedToOpen, //
                                             "failed to open file: " + path.generic_string()}};

    try
    {
        const auto fileSize = std::filesystem::file_size(path);
        std::vector<T> result(fileSize);
        file.seekg(0);
        file.read(reinterpret_cast<char*>(result.data()), fileSize);
        return result; // no need to call close, done by ifstream destructor
    }
    catch(...)
    {
        return std::unexpected{ReadFileError{ReadFileErrorCode::FailedToRead, //
                                             "failed to read file: " + path.generic_string()}};
    }
}

inline ReadFileResult<char> FileToCharVec(const std::filesystem::path& path)
{
    return FileToVec<char>(path); //
}

inline ReadFileResult<std::byte> FileToByteVec(const std::filesystem::path& path)
{
    return FileToVec<std::byte>(path); //
}

enum class SaveFileErrorCode
{
    FailedToOpen,
    Other
};

using SaveFileError = ::vic::StaticError<SaveFileErrorCode>;
using SaveFileResult = std::expected<void, SaveFileError>;

inline SaveFileResult CharVecToFile(const std::filesystem::path& path, //
                                    const FileData<char>& data)
{
    try
    {
        std::ofstream output(path, std::ios::out | std::ios::binary);
        if(!output.is_open())
            return std::unexpected{SaveFileError{SaveFileErrorCode::FailedToOpen, "failed to open file: " + path.generic_string()}};
        output.write((const char*)&data[0], data.size());
    }
    catch(...)
    {
        return std::unexpected{SaveFileError{SaveFileErrorCode::Other, //
                                             "failed to save file: " + path.generic_string()}};
    }
}

} // namespace vic