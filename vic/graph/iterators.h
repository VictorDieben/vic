#pragma once

#include "graph.h"


namespace vic
{
namespace graph
{

// iterate over all out edges of a certain vertex
template <typename TGraph, bool directed = false>
class OutIterator {};

// iterate over all out edges, for tensor vertices
template <typename TGraph, bool directed = false>
class TensorOutIterator {};

// iterate over all out edges, for tensor vertices, 
// such that no two agents are at the same spot, 
// and the transition is valid
template <typename TGraph, bool directed = false>
class TensorUniqueOutIterator {};

}
}