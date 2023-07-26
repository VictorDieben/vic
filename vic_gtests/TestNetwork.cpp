
#include "gtest/gtest.h"

#include "vic/network/binary_conversion.h"

namespace vic::network
{

TEST(Network, fundamental)
{
    std::array<Byte, 10000> buffer;

    const double someDouble = 1.2345;
    BinaryConversion<double>::Write(someDouble, buffer.begin(), buffer.end());
}
} // namespace vic::network