#pragma once

namespace vic
{
namespace graph2
{

template <typename T>
concept ConceptGraph = requires(T& graph) {
    typename T::VertexIdType;
    typename T::EdgeIdType;
    {
        graph.NumVertices()
    } -> std::integral;
    {
        graph.NumEdges()
    } -> std::integral;
};

// todo: replace with std version if it ever gets added to the standard
template <typename T>
concept arithmetic = std::is_arithmetic_v<T>;

template <typename T>
concept ConceptGraphEdgeList = ConceptGraph<T> && requires(T& graph) {
    {
        graph.Edges().begin()
    } -> std::forward_iterator;
    {
        graph.Edges().end()
    } -> std::forward_iterator;
};

template <typename T>
concept ConceptGraphCartesian = ConceptGraph<T> && requires(T& graph) {
    typename T::BaseGraphType;
    {
        graph.NumDimensions()
    } -> std::integral;
};

template <typename T>
concept ConceptHeuristic = requires(T& heuristic) {
    {
        heuristic.Cost({}, {})
    } -> arithmetic;
};

template <typename T>
concept ConceptPolicy = requires(T& policy) { policy.Policy({}, {}); };

template <typename T>
concept ConceptEdgeCost = requires(T& edgeCost) {
    {
        edgeCost({}, {})
    } -> arithmetic;
};

} // namespace graph2
} // namespace vic