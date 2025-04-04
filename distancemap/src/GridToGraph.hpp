#ifndef DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_
#define DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "FlowField.hpp"
#include "GridTypes.hpp"

namespace GridToGraph {

const int NODE = 0x010000;
const int DEND = 0x020000;
const int EDGE = 0x040000;
const int XPND = 0x080000;

using namespace GridType;

// For INPUT GRID
const int EMPTY = 0x00;
const int PATH = 0x01; // NOTE: Must be 1 for dead end detection


struct GridPointInfo {
    int closestBaseNodeIdx = -1;
    int closestBaseEdgeIdx = -1;
    int closestAbstractEdgeIdx = -1;
    int closestAbstractNodeIdx = -1;
	int distanceToAbstractNode = std::numeric_limits<int>::max();
    int directionToFromNode = -1; // Index into the directions array
    int directionToToNode = -1;   // Index into the directions array
    double angleToFromNode = 0.0; // Granular angle to the "from" node in degrees
    double angleToToNode = 0.0;   // Granular angle to the "to" node in degrees
    bool hasFlowFieldCoverage = false; // If part of a FlowField
};

struct ZoneInfo
{
	std::vector<int> baseNodes;
	std::vector<int> baseEdgeIdxs;
    std::vector<int> adjacentZones;
};

// 2D grid of information for each cell to aid navigation
using ZoneGrid = std::vector<std::vector<GridPointInfo>>;

struct AbstractLevel
{
    std::vector<AbstractEdge> abstractEdges;
    std::vector<AbstractNode> abstractNodes;
    ZoneGrid zoneGrid;
	std::vector<ZoneInfo> zones;
};

// Map of ALL BaseFromIdx,BaseToIdx pairs returning the total length of path
// connecting node pair. i.e. the total length of all the paths to get from
// the first node to the second node.
using PathCostMap = std::unordered_map<std::pair<int, int>, int, PairHash>;

struct FallbackCell {
    int nextFlowX = 0;
    int nextFlowY = 0; // Closest flow field point
    int distance = -1;
};
using FallbackGrid = std::vector<std::vector<FallbackCell>>;


struct Graph {
    GridType::Grid infoGrid;
    FallbackGrid fallbackGrid;
    BaseGraph baseGraph;
    std::vector<FlowField::SparseFlowField> flowFields;
    PathCostMap pathCostMap;
    std::vector<Edge> baseEdges;
    std::vector<GridType::Point> baseNodes;
    std::vector<GridType::Point> deadEnds;
    std::vector<AbstractLevel> abstractLevels;
};

////////////////////////////////////////////////////////////////////////////////
//
// Floor must = PATH on input i.e. walkable, WALLS = EMPTY
//
Graph makeGraph(const GridType::Grid& floorGrid);

//
// Function to generate the navigation map
//
ZoneGrid generateNavigationGrid(
    const GridType::Grid& grid,
    const std::vector<Edge>& baseEdges,
    const std::vector<AbstractEdge>& abstractEdges,
    const std::vector<GridType::Point>& baseNodes,
    const std::vector<AbstractNode>& abstractNodes);

std::vector<GridType::Point> detectDeadEnds(const GridType::Grid& grid);

std::vector<GridType::Point> detectNodes(const GridType::Grid& grid);

std::vector<Edge> findEdges(const GridType::Grid& grid, const std::vector<GridType::Point>& nodes, const std::vector<GridType::Point>& deadEnds);

// Note: grid changed 2 = node, 3 = deadend after calling this
void updateGrid(GridType::Grid& grid, const std::vector<GridType::Point>& Nodes, const std::vector<GridType::Point>& DeadEnds);

// Note: grid 1=path, 2=node, 3=deadend
std::vector<GridType::Path> findPaths(const GridType::Grid& grid);

void expandPaths(GridType::Grid& grid);

PathCostMap computeAllPaths(const std::vector<Edge>& baseEdges, int numNodes);

const int WALL = 0x800000; /// Internal use only
const int BOUNDARY = 0x400000; /// Internal use only
}

#endif /* DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_ */
