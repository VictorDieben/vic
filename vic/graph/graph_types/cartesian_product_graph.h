#pragma once

#include "vic/graph/definitions.h"
#include "vic/graph/traits.h"

#include <tuple>

namespace vic
{
namespace graph
{

template <ConceptGraph... TGraphs>
class CartesianGraph
{
public:
    CartesianGraph() = default;
    using GraphTypes = std::tuple<TGraphs...>; // use a tuple to index into a list of types

private:
};

} // namespace graph
} // namespace vic