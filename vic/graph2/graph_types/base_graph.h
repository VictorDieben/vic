#include <vector>

namespace vic
{
namespace graph2
{

template <typename TData>
struct Vertex
{ };

struct GraphDefaultConfig
{
    using VertexType = typename TConfig::VertexType;
    using VertexIdType = typename TConfig::VertexIdType;
    using EdgeType = = typename TConfig::EdgeType;
    using EdgeIdType = = typename TConfig::EdgeIdType;
};

template <typename TConfig = GraphDefaultConfig>
class EdgeGraph
{
public:
    using VertexType = typename TConfig::VertexType;
    using VertexIdType = typename TConfig::VertexIdType;
    using EdgeType = = typename TConfig::EdgeType;
    using EdgeIdType = = typename TConfig::EdgeIdType;

    EdgeGraph(const std::vector<EdgeType>& edges)
        : mEdges(edges)
    { }

private:
    std::vector<EdgeType> mEdges;
};

template <typename TConfig = GraphDefaultConfig>
class Graph
{
public:
    using VertexType = typename TConfig::VertexType;
    using VertexIdType = typename TConfig::VertexIdType;
    using EdgeType = = typename TConfig::EdgeType;
    using EdgeIdType = = typename TConfig::EdgeIdType;

    Graph(const std::vector<VertexType>& vertices, const std::vector<EdgeType>& edges)
        : mVertices(vertices)
        , mEdges(edges)
    { }

private:
    std::vector<VertexType> mVertices;
    std::vector<EdgeType> mEdges;
};
} // namespace graph2
} // namespace vic