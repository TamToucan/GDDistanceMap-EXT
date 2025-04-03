#ifndef DISTANCEMAP_SRC_GRIDTYPES_HPP_
#define DISTANCEMAP_SRC_GRIDTYPES_HPP_

#include <vector>
#include <utility>

namespace GridType {

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;
using Path = std::vector<Point>;

    // Directions for 8 neighbouring cells
const static std::vector<GridType::Point> directions = {
		{0, -1}, {0, 1}, {-1, 0}, {1, 0}, // Up, Down, Left, Right
		{-1, -1}, {1, -1}, {1, 1}, {-1, 1} // Diagonals
};


struct Edge {
    int from, to;   // Indices of connected nodes or deadEnds
    bool toDeadEnd; // Whether `to` refers to a deadEnd
    GridType::Path path;      // Path between the points
};

struct AbstractNode {
    std::vector<int> baseNodes;   // Indices of nodes in the cluster
    GridType::Point center;        // Geometric center of the cluster
    int baseCenterNode;			  // index of closest baseNode
};

struct AbstractEdge {
    int from, to; // Indices of connected abstract nodes
    GridType::Path path;

    bool operator<(const AbstractEdge& other) const {
        if (from != other.from) return from < other.from;
        return to < other.to;
    }
};


using AbstractGraph = std::pair<std::vector<AbstractNode>, std::vector<AbstractEdge>>;

struct BaseGraphInfo {
    int neighbor;   // Neighboring base node index.
    int edgeIndex;  // Index into the edges vector.
    bool forward;   // True if neighbor is the TO and indexing by FROM.
    int cost;       // Cost is the length of the edge path.
};

//            |-----baseEdge-----|
// Graph of Base nodes -> Base Node and EdgeIdx and Cost
// The from->to and to->from is stored for each baseEdge
using BaseGraph = std::vector< std::vector<BaseGraphInfo> >;

std::vector<int> checkConnectivity(const BaseGraph& graph, int numBaseNodes);

BaseGraph buildBaseGraph(const std::vector<Edge>& edges, int numBaseNodes);

}

#endif /* DISTANCEMAP_SRC_GRIDTYPES_HPP_ */
