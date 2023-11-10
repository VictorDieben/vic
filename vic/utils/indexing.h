#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <cassert>
#include <cstdlib> // std::div
#include <numeric>

namespace vic
{
namespace indexing
{

template <uint8_t dimensions>
using Shape = std::array<uint32_t, dimensions>;

// todo: a constexpr version for the cases where we know the size at compile time

template <typename TShape, typename TIndex>
    requires std::integral<TShape> && std::integral<TIndex>
bool IsValidIndex(const std::vector<TShape>& shape, const std::vector<TIndex>& index)
{
    if(shape.empty())
        return false; // todo: should a 0-dimensional array evaluate to false? there is no possible value for any flat index, so I think yes. However, an empty index array could still be considered "correct"
    if(shape.size() != index.size())
        return false;
    for(std::size_t idx = 0; idx < shape.size(); ++idx)
        if(index[idx] >= shape[idx]) // index needs to be < shape in each dimension
            return false;
    return true;
}

template <typename TRes, typename TShape, typename TIndex>
    requires std::integral<TRes> && std::integral<TShape> && std::integral<TIndex>
constexpr TRes NdIndexToFlat(const std::vector<TShape>& shape, const std::vector<TIndex>& index)
{
    assert(IsValidIndex(shape, index) && "invalid nd-index for this shape!");
    TRes result = index.back();

    TRes blockSize = 1; // todo: blocksize can be precomputed, usefull if we are transforming large amounts of data

    // iterate backward over vector, skipping last
    for(int i = shape.size() - 2; i >= 0; --i)
    {
        blockSize *= shape[i + 1];
        result += index[i] * blockSize;
    }

    return result;
}

template <typename TShape, typename TIndex, typename TBuffer>
    requires std::integral<TShape> && std::integral<TIndex> && std::integral<TBuffer>
void FlatToNdIndex(const std::vector<TShape>& shape, //
                   const TIndex index,
                   std::vector<TBuffer>& buffer)
{
    buffer.resize(shape.size());

    TIndex value = index;

    TIndex quotient = 0;
    TIndex remainder = 0;

    uint64_t blocksize;

    for(std::size_t i = 0; i < shape.size() - 1; ++i)
    {
        blocksize = std::accumulate(shape.begin() + 1 + i, shape.end(), 1, std::multiplies<TBuffer>()); // todo: precompute?

        quotient = value / blocksize;
        remainder = value % blocksize;

        buffer[i] = quotient;
        value = remainder;
    }

    buffer[shape.size() - 1] = remainder;
}

template <typename TRes, typename TShape, typename TFlat>
    requires std::integral<TRes> && std::integral<TShape> && std::integral<TFlat>
std::vector<TRes> FlatToNdIndex(const std::vector<TShape>& shape, //
                                const TFlat index)
{
    std::vector<TRes> buffer;
    FlatToNdIndex(shape, index, buffer);
    return buffer; //
}

} // namespace indexing
} // namespace vic