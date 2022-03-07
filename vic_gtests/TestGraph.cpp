#include "pch.h"
#include "vic/Graph.h"

using namespace vic;

TEST(TestGraph, Startup)
{
	using Vertex = vic::Vertex<>;
	using Edge = vic::Edge<Vertex::IdType>; // edges need to know what the id type of vertices are
	using TestGraph = vic::SimpleGraph<Vertex, Edge>;

	TestGraph graph;

	for (int i = 0; i < 5; ++i)
		graph.AddVertex();

	EXPECT_EQ(graph.GetVertex(0).mId, 0);
	EXPECT_EQ(graph.GetVertex(4).mId, 4);

	// iterate over all vertices
	size_t sum = 0;
	graph.ForeachVertex([&](const Vertex& vertex) { sum += vertex.mId; });
	EXPECT_EQ(sum, 0 + 1 + 2 + 3 + 4);

	// add some edges
	//   3
	// 0 1 2
	//   4
	graph.AddEdge(graph.GetVertex(0), graph.GetVertex(1));
	graph.AddEdge(graph.GetVertex(1), graph.GetVertex(2));
	graph.AddEdge(graph.GetVertex(1), graph.GetVertex(3));
	graph.AddEdge(graph.GetVertex(4), graph.GetVertex(1));

	// iterate over all edges
	sum = 0;
	graph.ForeachEdge([&](const Edge& edge) {sum += edge.mId; });
	EXPECT_EQ(sum, 0 + 1 + 2 + 3);

	// test out edges
	OutEdgeIterator<TestGraph, true> outEdgeIterator(graph); // valid untill vertices/edges change in graph

	std::vector<Vertex::IdType> outVertices;
	sum = 0;
	outEdgeIterator.ForeachOutEdge(graph.GetVertex(1).mId,
		[&](Edge& edge) { outVertices.push_back(edge.mSink); sum += edge.mSink; }
	);

	EXPECT_EQ(outVertices.size(), 2);
	EXPECT_EQ(sum, 2 + 3);

	// test neighbours
	NeighbourIterator<TestGraph> neighbourIterator(graph);

	sum = 0;
	outVertices.clear();
	neighbourIterator.ForeachNeighbour(graph.GetVertex(1),
		[&](Vertex& vertex) {sum += vertex.mId; outVertices.push_back(vertex); }
	);

	EXPECT_EQ(outVertices.size(), 4);
	EXPECT_EQ(sum, 0 + 2 + 3 + 4);

	// test dijkstra solver
	DijkstraSolver<TestGraph> dijkstra(graph);
	auto path = dijkstra.Calculate(graph.GetVertex(0), graph.GetVertex(2));

	EXPECT_EQ(path.size(), 3);
	EXPECT_EQ(path.at(0), 0);
	EXPECT_EQ(path.at(1), 1);
	EXPECT_EQ(path.at(2), 2);
}
