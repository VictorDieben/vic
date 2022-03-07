#pragma once

#include <cstdint>
#include <vector>
#include <limits>
#include <map>
#include <algorithm>


namespace vic
{

// example of VertexData struct
struct EmptyVertexDataType
{
	using IdType = uint16_t; // supports ~65000 different indices
};

// example of VertexData struct
struct EmptyEdgeDataType
{
	using IdType = uint16_t; // supports ~65000 different indices
};


//
template <typename TData = EmptyVertexDataType>
struct Vertex
{
	using IdType = typename TData::IdType;

	Vertex() = default;
	Vertex(IdType id) : mId(id) {}

	IdType mId{};
	operator IdType() const { return mId; }
};

template <typename TVertexId, typename TData = EmptyEdgeDataType>
struct Edge
{
	using IdType = typename EmptyEdgeDataType::IdType;

	Edge() = default;
	Edge(const TVertexId source, const TVertexId sink, const IdType id)
		: mSource(source)
		, mSink(sink)
		, mId(id) {}

	TVertexId mSource{};
	TVertexId mSink{};
	IdType mId{};
	operator IdType() const { return mId; }
};


// Graph interface class that all graphs need to support
template <typename TVertex, typename TEdge, typename TGraph>
class BaseGraph
{
public:
	using VertexType = typename TVertex;
	using VertexId = typename VertexType::IdType;

	using EdgeType = typename TEdge;
	using EdgeId = typename EdgeType::IdType;

	using DerivedGraphType = typename TGraph; // crtp

	// adding/getting/removing edges/vertices
	VertexType& AddVertex() { return static_cast<TGraph*>(this)->AddVertex(); }
	EdgeType& AddEdge(const VertexType& source, const VertexType& sink) { return static_cast<TGraph*>(this)->AddEdge(const VertexType & source, const VertexType & sink); }

	VertexType& GetVertex(VertexId id) { return static_cast<TGraph*>(this)->GetVertex(VertexId id); }
	EdgeType& GetEdge(EdgeId id) { return static_cast<TGraph*>(this)->GetEdge(EdgeId id); }

	void RemoveVertex(VertexId id) { return static_cast<TGraph*>(this)->RemoveVertex(VertexId id); }
	void RemoveEdge(EdgeId id) { return static_cast<TGraph*>(this)->RemoveEdge(EdgeId id); }

	// functionality
	template <typename TFunctor>
	void ForeachVertex(TFunctor functor) { return static_cast<TGraph*>(this)->ForeachVertex(TFunctor functor); }
	template <typename TFunctor>
	void ForeachEdge(TFunctor functor) { return static_cast<TGraph*>(this)->ForeachEdge(TFunctor functor); }
};


template <typename TVertex, typename TEdge>
class SimpleGraph
	: public BaseGraph<TVertex, TEdge, SimpleGraph<TVertex, TEdge>>
{
public:
	// note: same types are defined here and in BaseGraph
	using BaseGraph = BaseGraph<TVertex, TEdge, SimpleGraph>;

	using VertexType = typename BaseGraph::VertexType;
	using VertexId = typename BaseGraph::VertexId;

	using EdgeType = typename  BaseGraph::EdgeType;
	using EdgeId = typename BaseGraph::EdgeId;

	VertexType& AddVertex()
	{
		mVertices.emplace_back(); // todo: arguments
		mVertices.back().mId = static_cast<VertexId>(mVertices.size() - 1);
		return mVertices.back();
	}
	EdgeType& AddEdge(const VertexType& source, const VertexType& sink)
	{
		mEdges.emplace_back(source.mId, sink.mId, static_cast<EdgeId>(mEdges.size()));
		return mEdges.back();
	}

	// todo: when removing vertices/edges, we need to update all other ids as well
	void RemoveVertex(VertexId id)
	{
		throw std::runtime_error("Not implemented yet!");
	}
	void RemoveEdge(EdgeId id)
	{
		throw std::runtime_error("Not implemented yet!");
	}

	VertexType& GetVertex(const VertexId id)
	{
		auto& vertex = mVertices.at(size_t(id));
		assert(vertex.mId == id); // vertices should be in order 0, 1, 2...
		return vertex;
	}

	EdgeType& GetEdge(EdgeId id)
	{
		auto& edge = mEdges.at(size_t(id));
		assert(edge.mId == id); // edges should be in order 0, 1, 2...
		return edge;
	}

	// todo: crtp
	size_t NumVertices() const
	{
		return mVertices.size();
	}
	size_t NumEdges() const
	{
		return mEdges.size();
	}

	template <typename TFunctor>
	void ForeachVertex(TFunctor functor)
	{
		for (auto& vertex : mVertices)
			functor(vertex);
	}

	template <typename TFunctor>
	void ForeachEdge(TFunctor functor)
	{
		for (auto& edge : mEdges)
			functor(edge);
	}

private:
	std::vector<VertexType> mVertices;
	std::vector<EdgeType> mEdges;
};

// This class stores the out edges of every vertex in a compact way, 
// to avoid iterating over all edges
template <typename TGraph, bool directed = false>
class OutEdgeIterator
{
public:
	using GraphType = TGraph;
	using VertexId = typename TGraph::BaseGraph::VertexId;
	using EdgeId = typename TGraph::BaseGraph::EdgeId;

	OutEdgeIterator(GraphType& graph)
		: mGraph(graph)
	{
		Update();
	}

	void Update()
	{
		mOutEdges.clear();
		mGraph.ForeachEdge([&](const GraphType::EdgeType& edge)
			{
				mOutEdges[edge.mSource].push_back(edge.mId);
				if (!directed) // for c++17: if constexpr
					mOutEdges[edge.mSink].push_back(edge.mId);
			});
	}

	template <typename TFunctor>
	void ForeachOutEdge(const VertexId vertex, TFunctor functor)
	{
		assert(mOutEdges.find(vertex) != mOutEdges.end());
		for (auto& edge : mOutEdges.at(vertex))
			functor(mGraph.GetEdge(edge));
	}

private:
	GraphType& mGraph;
	std::map <typename VertexId, std::vector<typename EdgeId>> mOutEdges;
};

// This class stores the out edges of every vertex in a compact way, 
// to avoid iterating over all edges
template <typename TGraph>
class NeighbourIterator
{
public:
	using GraphType = TGraph;
	using VertexId = typename TGraph::BaseGraph::VertexId;

	NeighbourIterator(GraphType& graph)
		: mGraph(graph)
	{
		Update();
	}

	void Update()
	{
		mNeighbours.clear();
		mGraph.ForeachEdge([&](const GraphType::EdgeType& edge)
			{
				mNeighbours[edge.mSource].push_back(edge.mSink);
				mNeighbours[edge.mSink].push_back(edge.mSource);
			});
	}

	template <typename TFunctor>
	void ForeachNeighbour(const VertexId vertex, TFunctor functor)
	{
		assert(mNeighbours.find(vertex) != mNeighbours.end());
		for (auto neighbour : mNeighbours.at(vertex))
			functor(mGraph.GetVertex(neighbour));
	}

private:
	GraphType& mGraph;
	std::map <typename VertexId, std::vector<typename VertexId>> mNeighbours;
};


// Dijkstras algorithm using this graph class
template <typename TGraph, bool directed = false>
class DijkstraSolver
{
public:
	using GraphType = TGraph;
	using VertexType = typename TGraph::BaseGraph::VertexType;
	using VertexId = typename TGraph::BaseGraph::VertexId;
	using EdgeType = typename TGraph::BaseGraph::EdgeType;

	DijkstraSolver(GraphType& graph)
		: mGraph(graph)
		, mOutEdges(graph)
	{
	}

	std::vector<VertexId> Calculate(const VertexId start, const VertexId target)
	{
		using pairType = std::pair<uint32_t, VertexId>;
		auto compareLambda = [](const pairType& first, const pairType& second) { return first.first < second.first; };

		// initialize
		const size_t n = mGraph.NumVertices();
		std::vector<uint32_t> dist(n);
		std::vector<VertexType*> prev(n); // we store a ptr, to avoid having to use optional<int>
		std::vector<pairType> heap;

		mGraph.ForeachVertex(
			[&](VertexType& vertex) {
				dist.at(vertex) = std::numeric_limits<uint32_t>::max();
				prev.at(vertex) = nullptr;
			}
		);
		dist.at(start) = 0;
		heap.emplace_back(0, start);

		// iterate until we find target
		pairType current;
		while (!heap.empty())
		{
			current = heap.front();
			if (current.second == target)
				break;

			std::pop_heap(heap.begin(), heap.end(), compareLambda);
			heap.pop_back(); // place best item at back and then remove it

			mOutEdges.ForeachOutEdge(current.second,
				[&](EdgeType& edge)
				{
					VertexId other = edge.mSource == current.second ? edge.mSink : edge.mSource;
					uint32_t newcost = current.first + 1; // todo: edge cost
					if (newcost < dist.at(other))
					{
						dist.at(other) = newcost;
						prev.at(other) = &mGraph.GetVertex(current.second);

						heap.emplace_back(newcost, other);
						std::push_heap(heap.begin(), heap.end());
					}
				}
			);
		}

		if (current.second != target)
			return {}; // we don't want to throw, and this can happen if no path is possible

		// backtrack path
		std::vector<VertexId> path;
		path.emplace_back(target);
		VertexId currentVertexId = target;

		while (true)
		{
			VertexId previous = prev.at(currentVertexId)->mId;
			currentVertexId = previous;
			path.push_back(currentVertexId);
			if (currentVertexId == start)
				break;
			if (path.size() > n + 1)
			{
				// if this happens, there is something wrong in the algorithm
				throw std::runtime_error("Calculated path is larger than the number of vertices!");
			}
		}
		std::reverse(path.begin(), path.end());
		return path;
	}
private:
	GraphType& mGraph;
	OutEdgeIterator<GraphType, directed> mOutEdges;
};

} // namespace vic