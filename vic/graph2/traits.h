
namespace vic
{
namespace graph2
{

// note: any data can be a vertex right now
//template <typename T>
//concept ConceptVertex = requires { T::VertexIdType; };

template <typename T>
concept ConceptEdge = requires(T edge) {
                          edge.Source();
                          edge.Sink();
                      };

template <typename T>
concept ConceptGraph = requires(T graph) {
                           T::VertexType;
                           T::VertexIdType;
                           T::EdgeType;
                           T::EdgeIdType;
                       };

} // namespace graph2
} // namespace vic