#pragma once

#include "definitions.h"
#include <cstdint>

namespace vic
{
namespace graph
{

template <typename T>
concept ConceptVertex = requires { T::VertexIdType; };

template <typename T>
concept ConceptEdge = requires {
                          T::VertexIdType;
                          T::EdgeIdType;
                      };

template <typename T>
concept ConceptGraph = requires(T graph) {
                           T::VertexType;
                           T::VertexIdType;
                           T::EdgeType;
                           T::EdgeIdType;
                           graph.GetVertex(typename T::VertexIdType{});
                           graph.GetEdge(typename T::EdgeIdType{});
                           graph.GetEdge(typename T::VertexIdType{}, typename T::VertexIdType{});
                           graph.GetNumVertices();
                           graph.GetNumEdges();
                       };

// todo: tensor vertex concept
// todo: tensor edge concept
// todo: tensor graph concept
template <typename T>
concept ConceptTensorGraph = ConceptGraph<T> && requires(T graph) {
                                                    T::TensorVertexIdType;
                                                    T::TensorEdgeIdType;
                                                };

template <typename TVertex>
struct vertex_index
{
    using type = uint64_t;
};

template <typename TVertex>
using vertex_index_t = vertex_index<TVertex>::type;

template <typename TEdge>
struct edge_index
{
    using type = uint64_t;
};

template <typename TEdge>
using edge_index_t = edge_index<TEdge>::type;

} // namespace graph
} // namespace vic