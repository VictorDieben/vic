#include "pch.h"

#include "vic/network/binary_conversion.h"

using namespace vic::network;

TEST(TestNetwork, fundamental)
{
    std::array<Byte, 10000> buffer;

    const double someDouble = 1.2345;
    BinaryConversion<double>::Write(someDouble, buffer.begin(), buffer.end());
}