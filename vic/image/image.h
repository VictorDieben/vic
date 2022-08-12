#pragma once

#include <array>
#include <fstream>
#include <iostream>

#include "vic/linalg/matrices_dynamic.h"

namespace vic
{
namespace image
{

enum class EPixelFormat
{
    RGB,
    RGBA,
    YUV
};

// todo: image concept
using uchar = unsigned char;
using Color = std::array<uchar, 4>;

// Image that uses a matrix behind the scenes
class Image
{
public:
    Image(const std::size_t width, const std::size_t height)
        : mMatrix(height, width)
    { }

    constexpr EPixelFormat PixelFormat() const { return EPixelFormat::RGB; }

    Color Get(const std::size_t nx, const std::size_t ny) const
    {
        return mMatrix.Get(ny, nx); //
    }

    void Set(const std::size_t nx, const std::size_t ny, const Color& color)
    {
        mMatrix.At(ny, nx) = color; //
    }

    std::size_t Width() const { return mMatrix.GetColumns(); }
    std::size_t Height() const { return mMatrix.GetRows(); }

private:
    vic::linalg::MatrixDynamic<Color> mMatrix;
};

enum class ESaveStatus
{
    OK,
    ERROR
};

static constexpr std::array<unsigned char, 14> BmpFileHeaderTemplate{'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};

constexpr std::array<unsigned char, 14> GetBMPFileHeader(const int nx, const int ny)
{
    int filesize = 54 + (3 * nx * ny);
    auto header = BmpFileHeaderTemplate;
    header[2] = (unsigned char)(filesize);
    header[3] = (unsigned char)(filesize >> 8);
    header[4] = (unsigned char)(filesize >> 16);
    header[5] = (unsigned char)(filesize >> 24);
    return header;
}

static constexpr std::array<unsigned char, 40> BmpInfoHeaderTemplate{40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0};
constexpr std::array<unsigned char, 40> GetBMPInfoHeader(const int nx, const int ny)
{
    auto info = BmpInfoHeaderTemplate;
    info[4] = (unsigned char)(nx);
    info[5] = (unsigned char)(nx >> 8);
    info[6] = (unsigned char)(nx >> 16);
    info[7] = (unsigned char)(nx >> 24);
    info[8] = (unsigned char)(ny);
    info[9] = (unsigned char)(ny >> 8);
    info[10] = (unsigned char)(ny >> 16);
    info[11] = (unsigned char)(ny >> 24);
    return info;
}

template <typename TImage>
ESaveStatus SaveBMP(const std::string& path, const TImage& image)
{
    std::fstream output(path, std::ios::out | std::ios::binary | std::ios::trunc);
    if(!output)
        return ESaveStatus::ERROR;

    const auto header = GetBMPFileHeader((int)image.Width(), (int)image.Height());
    output.write((char*)&header[0], header.size());

    const auto info = GetBMPInfoHeader((int)image.Width(), (int)image.Height());
    output.write((char*)&info[0], info.size());

    // const uchar padding[3] = {0, 0, 0};

    for(std::size_t j = 0; j < image.Height(); ++j)
    {
        for(std::size_t i = 0; i < image.Width(); ++i)
        {
            const auto color = image.Get(i, j);
            output << color[2] << color[1] << color[0]; //  << padding;
        }
    }

    return ESaveStatus::OK;
}

template <typename TImage>
TImage LoadBMP(const std::string& path)
{
    std::basic_ifstream<uchar> file(path, std::ios::binary);

    // read entire file to a vector of bytes
    const std::vector<uchar> data((std::istreambuf_iterator<uchar>(file)), std::istreambuf_iterator<uchar>());

    // todo: verify that it is actually a bmp

    int width = *(int*)&data[18];
    int height = *(int*)&data[22];

    TImage image{std::size_t(width), std::size_t(height)};

    std::size_t idx = 54;
    for(std::size_t j = 0; j < height; ++j)
    {
        for(std::size_t i = 0; i < width; ++i)
        {
            const uchar b = data[idx];
            const uchar g = data[idx + 1];
            const uchar r = data[idx + 2];
            Color color{r, g, b, 255};
            image.Set(i, j, color);
            idx += 3; // r,g,b
        }
    }

    return image; //
}

} // namespace image
} // namespace vic