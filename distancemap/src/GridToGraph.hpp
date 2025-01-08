#ifndef DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_
#define DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_

#include <vector>
#include <utility>

namespace GridToGraph {

const int EMPTY = 0x00;
const int PATH = 0x01; // NOTE: Must be 1 for dead end detection

const int NODE = 0x02;
const int DEND = 0x03;
const int XPND = 0x04;

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;
using Path = std::vector<Point>;

struct Edge {
    int from, to;   // Indices of connected nodes or deadEnds
    bool toDeadEnd; // Whether `to` refers to a deadEnd
    Path path;      // Path between the points
};

struct AbstractNode {
    int id;                       // ID of the cluster
    std::vector<int> baseNodes;   // Indices of nodes in the cluster
    std::vector<int> baseDeadEnds; // Indices of deadEnds in the cluster
    Point center;                 // Geometric center of the cluster
};

struct AbstractEdge {
    int from, to; // Indices of connected abstract nodes
    std::vector<int> baseEdges;
    Path path;
};


using AbstractGraph = std::pair<std::vector<AbstractNode>, std::vector<AbstractEdge>>;


//
// Floor must = PATH on input i.e. walkable, WALLS = EMPTY
//
void makeGraph(const Grid& floorGrid);

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


}

#endif /* DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_ */
