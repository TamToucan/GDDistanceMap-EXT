#ifndef DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_
#define DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_

#include <vector>
#include <unordered_map>
#include <utility>

namespace GridToGraph {

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};


const int EMPTY = 0x00;
const int PATH = 0x01; // NOTE: Must be 1 for dead end detection

const int NODE = 0x02;
const int DEND = 0x03;
const int XPND = 0x04;
const int WALL = 0xff; /// Internal use only

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;
using Path = std::vector<Point>;

struct Edge {
    int from, to;   // Indices of connected nodes or deadEnds
    bool toDeadEnd; // Whether `to` refers to a deadEnd
    Path path;      // Path between the points
};

// Represents a precomputed path between two nodes
struct PrecomputedPath {
    int fromNode;
    int toNode;
    Path path;
};


struct AbstractNode {
    int id;                       // ID of the cluster
    std::vector<int> baseNodes;   // Indices of nodes in the cluster
    std::vector<int> baseDeadEnds; // Indices of deadEnds in the cluster
    Point center;                 // Geometric center of the cluster
    int baseCenterNode;			  // index of closest baseNode
};

struct AbstractEdge {
    int from, to; // Indices of connected abstract nodes
    std::vector<int> baseEdges;
    Path path;
};


using AbstractGraph = std::pair<std::vector<AbstractNode>, std::vector<AbstractEdge>>;

struct GridPointInfo {
    int closestBaseEdgeIdx = -1;
    int closestAbstractEdgeIdx = -1;
    int directionToFromNode = -1; // Index into the directions array
    int directionToToNode = -1;   // Index into the directions array
    double angleToFromNode = 0.0; // Granular angle to the "from" node in degrees
    double angleToToNode = 0.0;   // Granular angle to the "to" node in degrees
};
using NavGrid = std::vector<std::vector<GridPointInfo>>;

using PathCostMap = std::unordered_map<std::pair<int, int>, int, PairHash>;

struct Graph {
    Grid infoGrid;
    NavGrid navGrid;
    PathCostMap pathCostMap;
    std::vector<Edge> baseEdges;
    std::vector<AbstractEdge> abstractEdges;
    std::vector<Point> baseNodes;
    std::vector<AbstractNode> abstractNodes;
    std::vector<Point> deadEnds;
};

////////////////////////////////////////////////////////////////////////////////
//
// Floor must = PATH on input i.e. walkable, WALLS = EMPTY
//
Graph makeGraph(const Grid& floorGrid);

//
// Function to generate the navigation map
//
NavGrid generateNavigationGrid(
    const Grid& grid,
    const std::vector<Edge>& baseEdges,
    const std::vector<AbstractEdge>& abstractEdges,
    const std::vector<Point>& baseNodes,
    const std::vector<AbstractNode>& abstractNodes);

std::vector<Point> detectDeadEnds(const Grid& grid);

std::vector<Point> detectNodes(const Grid& grid);

std::vector<Edge> findEdges(const Grid& grid, const std::vector<Point>& nodes, const std::vector<Point>& deadEnds);

// Note: grid changed 2 = node, 3 = deadend after calling this
void updateGrid(Grid& grid, const std::vector<Point>& Nodes, const std::vector<Point>& DeadEnds);

// Note: grid 1=path, 2=node, 3=deadend
std::vector<Path> findPaths(const Grid& grid);

void thickenPaths(Grid& grid);

AbstractGraph createAbstractGraph(const std::vector<Point>& nodes, const std::vector<Point>& deadEnds,
								  const std::vector<Edge>& edges, double clusteringEps, int minClusterSize);


PathCostMap computeAllPaths(const std::vector<Edge>& baseEdges, int numNodes);

}

#endif /* DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_ */
