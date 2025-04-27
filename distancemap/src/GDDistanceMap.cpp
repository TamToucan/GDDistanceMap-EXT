#include <vector>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <limits>


#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>
#include <GDDistanceMap.hpp>

#include "GridToGraph.hpp"
#include "FlowField.hpp"
#include "Routing.hpp"
#include "GridTypes.hpp"
#include "MathUtils.h"

using namespace godot;

void GDDistanceMap::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_floor", "floorTileCoords"), &GDDistanceMap::setFloor);
	ClassDB::bind_method(D_METHOD("set_cave_size", "caveSize"), &GDDistanceMap::setCaveSize);
	ClassDB::bind_method(D_METHOD("set_cell_size", "cellSize"), &GDDistanceMap::setCellSize);
	ClassDB::bind_method(D_METHOD("make_distance_map", "pTileMap", "layer"), &GDDistanceMap::make_it);
	ClassDB::bind_method(D_METHOD("get_move", "from", "to", "type"), &GDDistanceMap::getMove);

}

GDDistanceMap::GDDistanceMap() {
	info.mFloor = Vector2i(0,0);
	info.mWall = Vector2i(0,1);
}

GDDistanceMap::~GDDistanceMap() {
}


GDDistanceMap* GDDistanceMap::setCaveSize(godot::Vector2i sz) {
	info.mCaveWidth = sz.width;
	info.mCaveHeight = sz.height;
	std::cerr << "SET CAVE: " << info.mCaveWidth << "x" << info.mCaveHeight << std::endl;
	return this;
}

GDDistanceMap* GDDistanceMap::setCellSize(godot::Vector2i sz) {
	info.mCellWidth = sz.width;
	info.mCellHeight = sz.height;
	std::cerr << "CELL: " << info.mCellWidth << "x" << info.mCellHeight << std::endl;
	return this;
}

GDDistanceMap* GDDistanceMap::setFloor(godot::Vector2i floor) {
	info.mFloor = floor;
	std::cerr << "FLOOR: " << info.mFloor.x << "x" << info.mFloor.y << std::endl;
	return this;
}

void GDDistanceMap::make_it(TileMapLayer* pTileMap, int layer)
{
	info.pTileMap = pTileMap;
	info.mLayer = layer;

	std::cerr << "=== COPY TILEMAP " << info.mCaveWidth << "x" << info.mCaveHeight
			<< " border:" << info.mBorderWidth << "," << info.mBorderHeight
			<< std::endl;

    std::vector<std::vector<int>> grid;
    for (int y=0; y < info.mCaveHeight+info.mBorderHeight*2; ++y) {
    	std::vector<int> row;
    	for (int x=0; x < info.mCaveWidth+info.mBorderWidth*2; ++x) {
    		Vector2i coords = getMapPos(x,y);
			int v = (info.pTileMap->get_cell_atlas_coords(coords) == info.mFloor) ? 0 : 1;
    		row.push_back(v);
    	}
    	grid.push_back(row);
    }

    //
    // Make a grid of distance to closest wall (0 = wall)
    //
    wallDistGrid = DistanceMap::makeWallDistanceGrid(grid);

    //
    // Make a SightGrid which can return the distance to wall (N,S,E,W)
    //
    sightGrid = DistanceMap::makeSightGrid(grid);

    //
    // Create a floor grid where
    //   WALLS = EMPTY
    //   FLOOR (i.e. non-zero distance to wall) = PATH
    //
    GridType::Grid floorGrid;
    for (const std::vector<int>& row : wallDistGrid) {
    	std::vector<int> floorRow;
    	for (int xy : row) {
    		floorRow.push_back(xy ? GridToGraph::PATH : GridToGraph::EMPTY);
    	}
    	floorGrid.push_back(floorRow);
    }

    //
    // Use that floorGrid to create the complete graph for movement
    //
    graph = GridToGraph::makeGraph(floorGrid);
}

// ===========================================================================

bool GDDistanceMap::isFloor(int cx, int cy, TileMapLayer* pTileMap, int layer) {
	if (cx >= 0 && cx < info.mCaveWidth && cy >= 0 && cy < info.mCaveHeight) {
		Vector2i coords = getMapPos(cx,cy);
		return (pTileMap->get_cell_atlas_coords(coords) == info.mFloor);
	}
	return false;
}

// ===========================================================================

Vector2i GDDistanceMap::getMapPos(int x, int y) {
	return Vector2i(info.mBorderWidth+x*info.mCellWidth, info.mBorderHeight+y*info.mCellHeight);
}

/////////////////////////////////////////////////////////

// Define constants
const int INVALID_DIRECTION = -1;
const int MAX_COST = 31;

// Utility functions
bool isValidGridPoint(const GridType::Grid& grid, const GridType::Point& point) {
    return point.first >= 0 && point.second >= 0 &&
           point.first < grid.size() && point.second < grid[0].size();
}

int manhattanDistance(const GridType::Point& a, const GridType::Point& b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

std::pair<int,int> getDxDy(GridType::Point fromPos, GridType::Point nextPos) {
	// Compute movement direction
	GridType::Point direction = {nextPos.first - fromPos.first, nextPos.second - fromPos.second};
	int dx = (direction.first == 0) ? 0 : (direction.first / std::abs(direction.first));
	int dy = (direction.second == 0) ? 0 : (direction.second / std::abs(direction.second));
	return {dx, dy};
}


double computeAngle(double dx, double dy) {
    return std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0);
}
// Helper function to compute angle
double computeAngle(GridType::Point from, GridType::Point to) {
    double dx = to.first - from.first;
    double dy = to.second - from.second;
    return computeAngle(dx, dy);
}

int pickNum(int range, int rnd)
{
    if (range <= 0) {
        return 0;
    }
    // Use modulo to wrap `rnd` into the range [0, range-1]
    return ((rnd % range) + range) % range;
}

// returns <EDGE_HALF (if edge halfway)> | <edgeIdx> 
int getNextEdge(const GridType::Point& pos, std::vector<GridType::Edge> edges,
    const GridType::Grid& infoGrid, const Routing::SparseGraph& routeGraph)
{
    std::cerr << "====GetNextEdge: " << pos.first << "," << pos.second << std::endl;
	int edgeIdx = routeGraph.edgeGrid.get(pos.first, pos.second) & 0xffff;
    int cell = infoGrid[pos.second][pos.first];
    std::cerr << "   => edgeGrid: " << edgeIdx << " infoGrid:" << std::hex << cell << std::dec << std::endl;
    if (cell & GridType::NODE) {
        int n = cell & 0xffff;
        const auto& connections = routeGraph.forward_adj[n];
        const int idx = pickNum(connections.size(), pos.first ^ pos.second);
        int edgeIdx = connections[idx].second;

        std::cerr << "      AT NODE. cons: " << connections.size()
    	<< " => idx:" << idx
    	<< " => edge: " << edgeIdx << std::endl;
        if (edges[edgeIdx].to == n) {
            std::cerr << " node:" << n << " is edge TO node => HALFWAY" << std::endl;
            edgeIdx |= GridType::EDGE_HALF;
        }
    }
    std::cerr << "   => " << (edgeIdx & GridType::EDGE_HALF ? "H " : "  ") << (edgeIdx & GridType::EDGE_MASK) << std::endl;
    return edgeIdx;
}

double routeToAngle(const GridToGraph::Graph& graph, const GridType::Point& source, const int srcEdgeIdx, const std::vector<int>& routeNodes)
{
    std::cerr << "##FIND ANGLE. edges:" << routeNodes.size() << std::endl;
	std::cerr << "ROUTE: " << std::endl;
    for (int n : routeNodes)
    {
        std::cerr << " " << n;
    }
	std::cerr << std::endl;

	int srcCell = graph.infoGrid[source.second][source.first];
	std::cerr << "  Source: " << source.first << "," << source.second << std::hex << " => " << srcCell << std::dec << std::endl;

	bool incPath = false;
	const auto& fromto = graph.routingGraph.edgeFromTos[srcEdgeIdx];
	std::cerr << "Checking edge: " << fromto.first << "->" << fromto.second << std::endl;
    if (srcCell & GridType::XPND) {
		int dirIdx = GridType::get_XPND_DIR(srcCell);
		GridType::Point dir = GridType::directions8[dirIdx];
        std::cerr << "  XPND: Use idx: " << dirIdx << " for dir " << dir.first << "," << dir.second << std::endl;
		return computeAngle(dir.first, dir.second);
    }
    else if (fromto.first ==  routeNodes.front()) {
        incPath = fromto.second == routeNodes[1];
        std::cerr << (incPath? "increase " : "decrease") << " path dist to TOnode: " << fromto.second << std::endl;
    }
    else if (fromto.second ==  routeNodes.front()) {
        incPath = ! fromto.first == routeNodes[1];
        std::cerr << (incPath? "increase " : "decrease") << " path dist to FROMnode: " << fromto.first << std::endl;
    }
    else if (srcCell & GridType::EDGE) {
		std::cout << "ERROR: " << source.first << "," << source.second << " on edge, but not start of path" << std::endl;
		std::cerr << "ERROR: " << source.first << "," << source.second << " on edge, but not start of path" << std::endl;
        incPath = (srcCell & GridType::EDGE_HALF);
    }
    else
    {
		std::cout << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
		std::cerr << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
    }

    int edgeCell = graph.routingGraph.edgeGrid.get(source.first, source.second);
	std::cerr << "  EdgeCell: " << std::hex << edgeCell << std::dec << std::endl;
	int dist = edgeCell >> 16;
	if (dist == 0)
	{
		if (routeNodes.size() == 1)
		{
			std::cout << "ERROR: " << source.first << "," << source.second << " edgeCell "
				<< std::hex << edgeCell << std::dec << ", dist 0, but path = 1" << std::endl;
			std::cerr << "ERROR: " << source.first << "," << source.second << " edgeCell "
				<< std::hex << edgeCell << std::dec << ", dist 0, but path = 1" << std::endl;
            return -1;
		}

        std::cerr << "  DIST 0: Check next node " << routeNodes[1] << std::endl;
		for (const auto& idx : graph.routingGraph.nodeToEdgeIdxs[routeNodes[1]])
		{
			const auto& ft = graph.routingGraph.edgeFromTos[idx];
            const auto& path = graph.baseEdges[idx].path;
			std::cerr << "  Node: " << routeNodes[1] << " gave edge:" << idx
				<< " " << ft.first << "->" << ft.second << ". Check for that node" << std::endl;
			if (ft.first == routeNodes[1]) {
				std::cerr << "    => Found " << routeNodes[1] << " move forward" << std::endl;
                return computeAngle(path.front(), path[1]);
			}
			else if (ft.second == routeNodes[1]) {
				std::cerr << "    => Found " << routeNodes[1] << " move backward" << std::endl;
                return computeAngle(path.back(), path[path.size() - 2]);
            }
		}
        std::cout << "ERROR: " << source.first << "," << source.second << " edgeCell "
            << std::hex << edgeCell << std::dec << ". At node, but no edge going to " << routeNodes[1] << std::endl;
        std::cerr << "ERROR: " << source.first << "," << source.second << " edgeCell "
            << std::hex << edgeCell << std::dec << ". At node, but no edge going to " << routeNodes[1] << std::endl;
		return -1;
	}
	const GridType::Edge& edge = graph.baseEdges[srcEdgeIdx];
	dist = std::clamp(dist + (incPath ? 1 : -1), 0, static_cast<int>(edge.path.size() - 1));
	auto movePnt = edge.path[dist];
	std::cerr << "  Use " << (incPath ? "increased" : "decrease") << " dist path[" << dist << "] ="
		<< movePnt.first << "," << movePnt.second << " move from: " << source.first<<","<<source.second << std::endl;

    return computeAngle(source,movePnt);
}

// Function to get the next move as an angle
double getNextMove(const GridToGraph::Graph& graph, GridType::Point& source, GridType::Point& target)
{
    int srcCell = graph.infoGrid[source.second][source.first];
    if (srcCell & GridType::WALL)
    {
		if (srcCell & GridType::BOUNDARY) {
            int dirIdx = srcCell & GridType::DIR_MASK;
            source.first += GridType::directions8[dirIdx].first;
            source.second += GridType::directions8[dirIdx].second;
            std::cerr << source.first << "," << source.second << " DIR: " << dirIdx << std::endl;
        }

        if (graph.infoGrid[source.second][source.first] & GridType::WALL) {
            std::cout << "ERROR: " << source.first << "," << source.second << " is WALL: SRC ***************************" << std::endl;
            std::cerr << "ERROR: " << source.first << "," << source.second << " is WALL: SRC ***************************" << std::endl;
			return 0;
        }
    }

    int tgtCell = graph.infoGrid[target.second][target.first];
    if (tgtCell & GridType::WALL)
    {
		if (tgtCell & GridType::BOUNDARY) {
            std::cerr << "MOVED TARGET " << target.first << "," << target.second << " => ";
            int dirIdx = tgtCell & GridType::DIR_MASK;
            target.first += GridType::directions8[dirIdx].first;
            target.second += GridType::directions8[dirIdx].second;
            std::cerr << target.first << "," << target.second << " DIR: " << dirIdx << std::endl;
        }

        if (graph.infoGrid[target.second][target.first] & GridType::WALL) {
            std::cout << "ERROR: " << target.first << "," << target.second << " is WALL: TARG ***************************" << std::endl;
            std::cerr << "ERROR: " << target.first << "," << target.second << " is WALL: TARG ***************************" << std::endl;
			return 0;
        }
    }

    // Check all the levels and find the level at which the zones are adjacent
    // - Not sure if could do top down search, I *think* it's possible to be
    // (say) adjacent at level 2, but same at level 1, because zones are any shape
    int zoneSame = -1;
    int zoneAdj = -1;
    for (int levelIdx=0; levelIdx < graph.abstractLevels.size(); ++levelIdx)
    {
        const auto& ablv = graph.abstractLevels[levelIdx];
        const auto& targetZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
        const auto& sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
        std::cerr << "LEVL: " << levelIdx << " src abNode: " << sourceZone << " dst abNode:" << targetZone << std::endl;
        // Check if in same zone for this level
        if (zoneSame == -1 && targetZone == sourceZone)
        {
            zoneSame = levelIdx;
            std::cerr << "==Zone Same: " << zoneSame << std::endl;
            continue;
        }

        // Check if target is one of the source neighbors
        for (int adjZone : ablv.zones[sourceZone].adjacentZones)
        {
            if (adjZone == targetZone)
            {
                zoneAdj = levelIdx;
				std::cerr << "==Zone ADJ: " << zoneAdj << std::endl;
                break;
            }
        }
        // If found adjacent then we have the lowest level that they are adjacent
        if (zoneAdj != -1)
        {
            break;
        }
    }

    // Check if found adjacent zones
    if (zoneAdj != -1) {
        const auto& ablv = graph.abstractLevels[zoneAdj];
        const auto& sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
        const auto& targetZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
        const auto& subgrid = ablv.subGrids[sourceZone];
        uint16_t sub = subgrid.getCostFlow(source.first, source.second, subgrid.getFlow(targetZone));
        const auto& dir = GridType::directions8[sub & 0xff];
		std::cerr << "  ADJ => sZ:" << sourceZone << " tZ: " << targetZone << " sub: " << std::hex << sub << std::dec
			<< " => dir: " << dir.first << "," << dir.second << std::endl;
        return computeAngle(dir.first, dir.second);

    }
    // No adjacent, check if found same zone
    int sourceEdge = -1;
    int targetEdge = -1;
    int levelIdx = -1;
    int zoneIdx = -1;
    if (zoneSame != -1) {
        levelIdx = zoneSame;
        const auto& ablv = graph.abstractLevels[levelIdx];
        zoneIdx = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;

        const auto& infoGrid = graph.infoGrid;

        // If on XPND then move towards edge
		int srcCell = infoGrid[source.second][source.first];
		if (srcCell & GridType::XPND) {
            int dirIdx = GridType::get_XPND_DIR(srcCell);
            GridType::Point dir = GridType::directions8[dirIdx];
			std::cerr << "  SAME src SRC => xy:" << source.first<<","<<source.second
			<< " cell:" << std::hex << srcCell << std::dec <<" dir:" << dirIdx
			<< " => " << dir.first << "," << dir.second << std::endl;
			return computeAngle( dir.first,dir.second );
		}

        // If not on EDGE then move towards edge path
        // Get the edges to route from/to and get the route
        sourceEdge = getNextEdge(source, graph.baseEdges, graph.infoGrid, graph.routingGraph);
        targetEdge = getNextEdge(target, graph.baseEdges, graph.infoGrid, graph.routingGraph);
        std::cerr << "  SAME => Route: " << sourceEdge << " to " << targetEdge << std::endl;
		// if both on the edge then move towards target
		if ((sourceEdge & GridType::EDGE_MASK) == (targetEdge&GridType::EDGE_MASK))
		{
            std::cerr << "ROUTE: SAME EDGE: " << (sourceEdge & GridType::EDGE_MASK) << std::endl;
			const auto& sourcePath = graph.baseEdges[sourceEdge & GridType::EDGE_MASK].path;
            int sourceDist = graph.routingGraph.edgeGrid.get(source.first,source.second) >> 16;
            int targetDist = graph.routingGraph.edgeGrid.get(target.first,target.second) >> 16;
            std::cerr << "  => sourceDist: " << sourceDist << " targetDist: " << targetDist << std::endl;
            sourceDist += (sourceDist < targetDist) ? 1 : -1;
            std::cerr << "  => Path[" << sourceDist << "] = "
                << sourcePath[sourceDist].first << ","<< sourcePath[sourceDist].second << std::endl;
			return computeAngle(source, sourcePath[sourceDist]);
		}
	}
	// Use last Abstract Level to route.
    else
    {
        std::cerr << "  LAST zone" << std::endl;
        levelIdx = graph.abstractLevels.size() - 1;
        const auto& ablv = graph.abstractLevels[levelIdx];
        zoneIdx = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
		const auto& sourceInfo = ablv.zones[zoneIdx];
        sourceEdge = getNextEdge(source, graph.baseEdges, graph.infoGrid, graph.routingGraph);

        GridType::AbstractNode abNode = ablv.abstractNodes[zoneIdx];

        std::cerr << "    Use zone: " << zoneIdx << " baseNode:" << abNode.baseCenterNode << " to find edge" << std::endl;
        const GridType::Point& targetPos = graph.baseNodes[abNode.baseCenterNode];
        targetEdge = getNextEdge(targetPos, graph.baseEdges, graph.infoGrid, graph.routingGraph);
    }
	// Use the source and target edges to get a route
    int srcEdgeIdx = (sourceEdge & GridType::EDGE_MASK);
    int dstEdgeIdx = (targetEdge & GridType::EDGE_MASK);
    std::cerr << "MOVE: zoneLevel: " << levelIdx << " zoneIndex: " << zoneIdx
        << " srcEdge:" << srcEdgeIdx << " (" << graph.baseEdges[srcEdgeIdx].from << " -> " << graph.baseEdges[srcEdgeIdx].to << ")" 
        << " dstEdge:" << dstEdgeIdx << " (" << graph.baseEdges[dstEdgeIdx].from << " -> " << graph.baseEdges[dstEdgeIdx].to << ")"
        << std::endl;

	const auto& ablv = graph.abstractLevels[levelIdx];
	const auto& sourceInfo = ablv.zones[zoneIdx];

	const auto& zoneBases = sourceInfo.baseNodeIdxs;
	const auto& zoneEdges = sourceInfo.baseEdgeIdxs;
	const auto routeNodes = Routing::findZonePath(graph.routingGraph,
		zoneBases, zoneEdges, srcEdgeIdx, dstEdgeIdx);
	return routeToAngle(graph, source, srcEdgeIdx, routeNodes);
}

int directionToDegrees(const GridType::Point p) {
    static const std::unordered_map<std::pair<int, int>, int, GridType::PairHash> directionMap = {
        {{1, 0}, 0},   // Right
        {{1, 1}, 45},  // Up-Right
        {{0, 1}, 90},  // Up
        {{-1, 1}, 135}, // Up-Left
        {{-1, 0}, 180}, // Left
        {{-1, -1}, 225}, // Down-Left
        {{0, -1}, 270},  // Down
        {{1, -1}, 315}   // Down-Right
    };

    auto it = directionMap.find(p);
    int dir = (it != directionMap.end()) ? it->second : -1; // Return -1 for invalid direction
    std::cerr << "===> " << p.first << "," << p.second << " => dir: " << dir << std::endl;
    return dir;
}

float GDDistanceMap::getMove(godot::Vector2 from, godot::Vector2 to, int type) {
	std::cerr << "********** FROM:" << from.x << "," << from.y << " TO " << to.x << "," << to.y << "   cell:" << info.mCellWidth << "x" << info.mCaveHeight << std::endl;
	GridType::Point fromPnt = {from.x/(info.mCellWidth*8), from.y/(info.mCellHeight*8) };
	GridType::Point toPnt = {to.x/(info.mCellWidth*8), to.y/(info.mCellHeight*8) };
	std::cerr << "===GETMOVE: " << fromPnt.first << "," << fromPnt.second << " TO " << toPnt.first << "," << toPnt.second << std::endl;
    float ang = static_cast<float>(getNextMove(graph, fromPnt, toPnt));
	std::cerr << "  RET " << ang << std::endl;
	return ang;
}

