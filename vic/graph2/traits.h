#pragma once

namespace vic
{
namespace graph2
{
     
template <typename T>
concept ConceptEdge = requires(T edge) {
                          edge.Source();
                          edge.Sink();
                      };

template <typename T>
concept ConceptGraph = requires(T& graph) {
                           typename T::VertexIdType;
                           typename T::EdgeIdType;
                           graph.NumVertices();
                           graph.NumEdges();
                       };

template <typename T>
concept ConceptGraphEdgeList = ConceptGraph<T> && requires(T& graph) { graph.Edges(); };

template <typename T>
concept ConceptGraphCartesian = ConceptGraph<T> && requires(T& graph) {
                                                       typename T::BaseGraphType;
                                                       graph.NumDimensions();
                                                   };

} // namespace graph2
} // namespace vic