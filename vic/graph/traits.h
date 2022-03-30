#pragma once

namespace vic
{
namespace graph
{

template <typename T>
concept ConceptVertex = requires
{
    T::VertexIdType;
};

template <typename T>
concept ConceptEdge = requires
{
    T::VertexIdType;
    T::EdgeIdType;
};

template <typename T>
concept ConceptGraph = requires(T graph)
{
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

// todo(vicdie): tensor vertex concept
// todo(vicdie): tensor edge concept
// todo(vicdie): tensor graph concept
template <typename T>
concept ConceptTensorGraph = ConceptGraph<T> && requires(T graph)
{
    T::TensorVertexIdType;
    T::TensorEdgeIdType;
};

} // namespace graph
} // namespace vic