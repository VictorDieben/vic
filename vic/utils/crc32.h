#pragma once

#include <cstdint>
#include <vector>

namespace vic
{

uint32_t crc32(const char* data, const uint64_t length)
{
    static constexpr uint32_t initial = 0xFFFFFFFF;
    static constexpr uint32_t polynomial = 0xEDB88320;

    uint32_t tmp = initial;

    for(uint64_t i = 0; i < length; i++)
    {
        tmp ^= (unsigned char)data[i]; // todo: avoid zero extended requirement using tmp variable?
        for(uint64_t bit = 0; bit < 8; bit++)
            if(tmp & 0x01)
                tmp = (tmp >> 1) ^ polynomial;
            else
                tmp >>= 1;
    }
    return tmp ^ initial;
}

uint32_t crc32(const std::vector<char>& data)
{
    return crc32(data.data(), data.size()); //
}

} // namespace vic