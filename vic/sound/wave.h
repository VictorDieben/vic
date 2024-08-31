#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <span>
#include <vector>

namespace vic
{
namespace sound
{

// for now only .wav is supported

static constexpr std::size_t WaveHeaderSize = 44; // [bits]

static constexpr std::array<std::byte, 4> RiffMarker = {std::byte{'R'}, std::byte{'I'}, std::byte{'F'}, std::byte{'F'}};
static constexpr std::array<std::byte, 4> WaveMarker = {std::byte{'W'}, std::byte{'A'}, std::byte{'V'}, std::byte{'E'}};
static constexpr std::array<std::byte, 4> FmtMarker = {std::byte{'f'}, std::byte{'m'}, std::byte{'t'}, std::byte{' '}};
static constexpr std::array<std::byte, 4> DataMarker = {std::byte{'d'}, std::byte{'a'}, std::byte{'t'}, std::byte{'a'}};

using DataLength = uint32_t;
using FormatType = uint16_t;
using NumberOfChannels = uint16_t;
using SampeRate = uint32_t; // Number of Samples per second
using BitsPerSample = uint16_t;
using TotalBitrate = uint32_t; // todo
using DataSize = uint32_t; // data section size; todo: bytes or items?

struct WaveHeader
{
    // DataLength length;
    FormatType format;
    NumberOfChannels numberOfChannels;
    SampeRate sampleRate;
    BitsPerSample bytesPerSecond;
    TotalBitrate totalBitRate;
    // DataSize dataSize;
};

struct Wave
{
    WaveHeader header;
    std::vector<std::byte> data;
};

template <typename T>
// requires integer<T>
T FromSpan(const std::span<std::byte>& data)
{
    return *reinterpret_cast<T*>(data.data()); //
}

inline WaveHeader ReadWaveHeader(const std::span<std::byte>& data)
{
    assert(data.size() >= WaveHeaderSize);
    assert(std::equal(data.begin(), data.begin() + 4, RiffMarker.begin())); // assert begin == RIFF
    assert(std::equal(data.begin() + 8, data.begin() + 12, WaveMarker.begin()));

    WaveHeader header;
    // header.length = FromSpan<DataLength>(data.subspan(4, 4));
    header.format = FromSpan<FormatType>(data.subspan(20, 2));
    header.numberOfChannels = FromSpan<NumberOfChannels>(data.subspan(22, 2));
    header.sampleRate = FromSpan<SampeRate>(data.subspan(24, 4));
    header.bytesPerSecond = FromSpan<BitsPerSample>(data.subspan(34, 2));
    // header.totalBitRate = FromSpan<TotalBitrate>(data.subspan(4, 4));
    // header.dataSize = FromSpan<DataSize>(data.subspan(40, 4));

    return header;
}

constexpr float Float24To32(const unsigned char* src) // note: apparantly the conversion function for .wav files
{
    int i = ((src[2] << 24) | (src[1] << 16) | (src[0] << 8)) >> 8;
    return ((float)i) / 8388607.0;
}

inline Wave ReadWave(const std::span<std::byte>& data)
{
    assert(data.size() >= WaveHeaderSize);

    Wave wave;
    wave.header = ReadWaveHeader(data);

    // ignore filesize according to header

    const auto vecdata = std::vector<std::byte>{data.begin() + WaveHeaderSize, data.end()};

    // todo: read data in blocks, create a vec of audio tracks or something

    // assert(data.size() == wave.header.

    return wave;
}

} // namespace sound
} // namespace vic