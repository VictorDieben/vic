#pragma once

#include <array>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "vic/utils/file.h"

namespace vic
{
namespace image
{

enum class EPixelFormat
{
    Grey,
    RG,
    RGB,
    RGBA,
    YUV
};

constexpr uint8_t PixelColorDimensions(const EPixelFormat format)
{
    switch(format)
    {
    case EPixelFormat::Grey:
        return 1;
    case EPixelFormat::RG:
        return 2;
    case EPixelFormat::RGB:
        return 3;
    case EPixelFormat::RGBA:
        return 4;
    case EPixelFormat::YUV:
        return 3;
    default:
        return 0; // invalid format
    }
}

template <typename T>
concept ConceptImage = requires(T image) {
    image.Width();
    image.Height();
};

using uchar = unsigned char; // uint8_t ?

template <uint64_t dims>
using Color = std::array<uchar, dims>;

using Color1 = Color<1>;
using Color2 = Color<2>;
using Color3 = Color<3>;
using Color4 = Color<4>;

using Resolution = uint32_t;

struct Bitmap
{
    Bitmap(const Resolution x, const Resolution y)
        : mX(x)
        , mY(y)
        , mData(mX * mY)
    { }

    Resolution Width() const { return mX; }
    Resolution Height() const { return mY; }

    const Color3& GetColor(const Resolution x, const Resolution y) const
    {
        assert(x < mX && y < mY);
        return mData[x + (mX * y)];
    }
    void SetColor(const Resolution x, const Resolution y, const Color3& color)
    {
        assert(x < mX && y < mY);
        mData[x + (mX * y)] = color;
    }

private:
    Resolution mX;
    Resolution mY;

    std::vector<Color3> mData;
};

inline bool IsBMP(const std::vector<char>& data)
{
    if(data[0] != 'B' || data[1] != 'M')
        return false;

    // todo: verify file size

    // todo: verify that bit depth is valid for bmp

    return true;
}

inline constexpr int BMPRowPadding(const Resolution nx) { return (4 - (nx * 3) % 4) % 4; }

inline std::optional<Bitmap> LoadBMP(const std::filesystem::path& path)
{
    const auto opt = vic::FileToCharVec(path);
    if(!opt)
        return std::nullopt;
    const auto& data = *opt;

    if(!IsBMP(data))
        return std::nullopt;

    const int filesize = *(int*)&data[2];
    const int width = *(int*)&data[18];
    const int height = *(int*)&data[22];

    const int paddingSize = BMPRowPadding(width);

    Bitmap image{Resolution(width), Resolution(height)};

    std::size_t idx = 54; // header + info size
    for(std::size_t j = 0; j < height; ++j)
    {
        for(std::size_t i = 0; i < width; ++i)
        {
            const uchar b = data[idx];
            const uchar g = data[idx + 1];
            const uchar r = data[idx + 2];
            Color3 color{r, g, b};
            image.SetColor(i, j, color);
            idx += 3; // r,g,b
        }

        idx += paddingSize;
    }

    return image; //
}

enum class ESaveStatus
{
    OK,
    ERROR
};

inline constexpr auto GetBMPFileHeader(const Resolution nx, //
                                       const Resolution ny,
                                       const int padding)
{
    constexpr const int headerSize = 14;
    constexpr const int informationheaderSize = 40;

    const int filesize = headerSize + informationheaderSize + (3 * (int)(nx * ny)) + padding;
    std::array<uchar, headerSize> header{};
    header[0] = 'B';
    header[1] = 'M';
    header[2] = (uchar)(filesize);
    header[3] = (uchar)(filesize >> 8);
    header[4] = (uchar)(filesize >> 16);
    header[5] = (uchar)(filesize >> 24);
    // optional
    header[6] = 0; // Reserved; actual value depends on the application that creates the image, if created manually can be 0
    header[7] = 0;
    header[8] = 0;
    header[9] = 0;
    const int offset = 123; // todo: compute and write

    header[10] = headerSize + informationheaderSize;

    return header;
}

inline constexpr auto GetBMPInfoHeader(const int nx, //
                                       const int ny,
                                       const int bitsPerChannel)
{
    constexpr const int informationheaderSize = 40;

    std::array<uchar, informationheaderSize> info{};
    info[0] = informationheaderSize;
    info[4] = (uchar)(nx);
    info[5] = (uchar)(nx >> 8);
    info[6] = (uchar)(nx >> 16);
    info[7] = (uchar)(nx >> 24);
    info[8] = (uchar)(ny);
    info[9] = (uchar)(ny >> 8);
    info[10] = (uchar)(ny >> 16);
    info[11] = (uchar)(ny >> 24);

    info[12] = 1; // planes (?)

    info[14] = bitsPerChannel; // bits per channel

    return info;
}

inline ESaveStatus SaveBMP(const std::filesystem::path& path, const Bitmap& image)
{
    std::ofstream output(path, std::ios::out | std::ios::binary | std::ios::trunc);
    if(!output)
        return ESaveStatus::ERROR;

    try
    {
        const int paddingSize = BMPRowPadding(image.Width());
        const int bitsPerPixel = 24;

        const auto header = GetBMPFileHeader(image.Width(), image.Height(), paddingSize);
        output.write(reinterpret_cast<const char*>(&header), header.size());

        const auto info = GetBMPInfoHeader(image.Width(), image.Height(), bitsPerPixel);
        output.write(reinterpret_cast<const char*>(&info), info.size());

        const uchar padding[3] = {0, 0, 0};

        for(std::size_t j = 0; j < image.Height(); ++j)
        {
            for(std::size_t i = 0; i < image.Width(); ++i)
            {
                const auto& col = image.GetColor(i, j);
                const uchar color[] = {col[2], col[1], col[0]};
                output.write(reinterpret_cast<const char*>(&color), 3);
            }
            // padding
            output.write(reinterpret_cast<const char*>(&padding), paddingSize);
        }

        return ESaveStatus::OK;
    }
    catch(...)
    {
        return ESaveStatus::ERROR;
    }
}

} // namespace image
} // namespace vic