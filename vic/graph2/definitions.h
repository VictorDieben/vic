#pragma once

#include <cstdint>

namespace vic
{
namespace graph2
{

using VertexId = uint16_t; // todo:: make configurable

template <typename TVertexIdType>
using CartesianVertex = std::vector<TVertexIdType>;

} // namespace graph2
} // namespace vic