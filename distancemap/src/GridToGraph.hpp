#ifndef DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_
#define DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_

#include <vector>
#include <unordered_map>
#include <utility>

#include "FlowField.hpp"
#include "GridTypes.hpp"

#ifdef _WIN32
#ifdef GDDISTANCEMAP_EXPORTS  // Must match your project name
#define GD_API __declspec(dllexport)
#else
#define GD_API __declspec(dllimport)
#endif
#else
#define GD_API
#endif

//
// Take the 2D grid of EMPTY/PATH (i.e. 1 = floor, 0 = wall) and
// generate the thinned (1 pixel) 2D grid from that. This gives
// the sarting point to
// - work out all the "intersection" (.baseNodes)
// - find all the deadends (.deadEnds)
// - find the list of paths connecting nodes/deadends (.baseEdges)
// Then find the nodes and paths for the "main routes" to create
// an AbstractLevel
// - Repeat generating AbstractLevel, grouping increasing number
//   of baseNodes until small enough set of AbstractNodes
//  Each AbstractLevel has AbstractNodes which are connected
//  via AbstractEdges.
// An AbstractNode is a grouping of nearyby BaseNodes with the
// index of the .baseNodes for it's point.
// The AbstractEdges are the paths through the bottom level
// graph of baseNodes and baseEdges that connect nearby
// AbstractNodes
//
// So there is complex graph of baseNodes and baseEdges with
// each AbtractLevel being the simplified graph made by
// grouping baseNodes together into AbstractNodes and
// AbstraceEdges being the paths through the base graph
// between those nodes.
//
// Then for each AbstractLevel the 2D grid is split int
// a voronoi like set of ZoneInfo where a zone is a single
// AbstractNode.
// Each ZoneInfo has the list of all the baseNodes and baseEdges in
// the zone and the list of AbstractNode indexes of the adjacent zones.
// - Each zone also has the lis of cells the form the boundary with a
//   neighboring zone that it's actually connected to.
// The AbstractLevel also has 
//
// ######################      4 Zones made for the 4 abstract nodes
// #            -       #      Zone0 adjacent to Zone1
// #     1      -       #      Zone1 adjacent to Zone0, Zone2
// #            -   2   #      Zone2 adjacent to Zone1, Zone3
// #-------#-----       #      Zone3 adjacent to Zone2
// #       #            #      The '-' are boundary cells
// #       ########-----#
// #   0   #            # 
// #       #    3       #
// #       #            #
// ######################
//
//

namespace GridToGraph {

GD_API std::vector<std::vector<int>> readGridFromFile(const std::string& filename);

using namespace GridType;

// For INPUT GRID
const int EMPTY = 0x00;
const int PATH = 0x01; // NOTE: Must be 1 for dead end detection

    // An AbstractLevel takes the base nodes and groups them into
    // clusters of size based on the level number. This creates
	// it's own craft of abstract nodes and edges and a grid of
	// ZonePointInfo.
	// Each abstract node is then turned into a ZoneInfo with
	// info on the base nodes/edges in that zone and the adjacent zones.
    //
struct AbstractLevel
{
    std::vector<AbstractEdge> abstractEdges;
    std::vector<AbstractNode> abstractNodes;
    ZoneGrid zoneGrid;
	std::vector<FlowField::SubGrid> subGrids;
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
    BaseGraph baseGraph;
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
GD_API Graph makeGraph(const GridType::Grid& floorGrid);

std::vector<GridType::Point> detectDeadEnds(const GridType::Grid& grid);

std::vector<GridType::Point> detectNodes(const GridType::Grid& grid);

std::vector<Edge> findEdges(const GridType::Grid& grid, const std::vector<GridType::Point>& nodes, const std::vector<GridType::Point>& deadEnds);

// Note: grid changed 2 = node, 3 = deadend after calling this
void updateGrid(GridType::Grid& grid, const std::vector<GridType::Point>& Nodes, const std::vector<GridType::Point>& DeadEnds);

// Note: grid 1=path, 2=node, 3=deadend
std::vector<GridType::Path> findPaths(const GridType::Grid& grid);

void expandPaths(GridType::Grid& grid);

PathCostMap computeAllPaths(const std::vector<Edge>& baseEdges, int numNodes);

}

#endif /* DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_ */
