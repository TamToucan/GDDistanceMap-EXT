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

int getNextEdge(const GridType::Point& pos, const GridType::Grid& infoGrid, const Routing::SparseGraph& routeGraph)
{
    std::cerr << "====GetNextEdge: " << pos.first << "," << pos.second << std::endl;
    int cell = infoGrid[pos.second][pos.first];
    if (cell & GridType::EDGE) {
        std::cerr << "      AT EDGE: " << (cell & GridType::EDGE_MASK) << std::endl;
        return cell & GridType::EDGE_MASK;
    }

    if (cell & GridType::NODE) {
        int n = cell & 0xffff;
        const auto& connections = routeGraph.forward_adj[n];
        const int idx = pickNum(connections.size(), pos.first ^ pos.second);
        std::cerr << "      AT NODE. cons: " << connections.size()
    	<< " => idx:" << idx
    	<< " => edge: " << connections[idx].second << std::endl;
        return connections[idx].second;
    }
    if (cell & GridType::DEND) {
        std::cerr << "      AT DEAD. Find dir" << std::endl;
        for (const auto [dx, dy] : GridType::directions8) {
            int nx = pos.first + dx;
            int ny = pos.second + dy;
            if (infoGrid[ny][nx] & GridType::EDGE)
            {
                std::cerr << "      => " << (infoGrid[ny][nx] & GridType::EDGE_MASK) << std::endl;
                return (infoGrid[ny][nx] & GridType::EDGE_MASK);
            }
        }
        std::cerr << "************** ERROR. DeadEnd " << pos.first << "," << pos.second << " has no edge" << std::endl;
    }
    if (cell & GridType::XPND) {
        int edgeIdx = GridType::get_XPND_EDGE(cell);
        std::cerr << "      XPND. edge: " << edgeIdx << std::endl;
        return edgeIdx;
    }
    std::cerr << "*******GetEdge: cell " << std::hex << cell << std::dec << std::endl;
	return -1;
}

double routeToAngle(const GridToGraph::Graph& graph, const std::vector<int>& routeEdges)
{
    std::cerr << "##FIND ANGLE. edges:" << routeEdges.size() << std::endl;
    return 0;
}

// Function to get the next move as an angle
double getNextMove(const GridToGraph::Graph& graph, const GridType::Point& source, const GridType::Point& target)
{
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
				std::cerr << "==Zone ADJ: " << zoneSame << std::endl;
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
        const auto& flow = subgrid.getFlow(targetZone);
        uint16_t sub = flow[FlowField::SubGrid::indexFor(source.first, source.second, subgrid.width)];
        const auto& dir = GridType::directions8[sub & 0xff];
		std::cerr << "  ADJ => sZ:" << sourceZone << " tZ: " << targetZone << " sub: " << std::hex << sub << std::dec << std::endl;
        return computeAngle(dir.first, dir.second);

    }
    // No adjacent, check if found same zone
    if (zoneSame != -1) {
        const auto& ablv = graph.abstractLevels[zoneSame];
        const auto& sameZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
        const auto& zoneInfo = ablv.zones[sameZone];
        const auto& zoneBases = zoneInfo.baseNodeIdxs;
        const auto& zoneEdges = zoneInfo.baseEdgeIdxs;

        const auto& infoGrid = graph.infoGrid;

        // If not on EDGE then move towards edge path
		int nx = source.first;
		int ny = source.second;
		int cell = infoGrid[ny][nx];
		if (cell & GridType::XPND) {
            int dirIdx = GridType::get_XPND_DIR(cell);
            GridType::Point dir = GridType::directions8[dirIdx];
			std::cerr << "  SAME src SRC => xy:" << nx<<","<<ny
			<< " cell:" << std::hex << cell << std::dec <<" dir:" << dirIdx
			<< " => " << dir.first << "," << dir.second << std::endl;
			return computeAngle( dir.first,dir.second );
		}

        // Get the edges to route from/to and get the route
        int sourceEdge = getNextEdge(source, infoGrid, graph.routingGraph);
        int targetEdge = getNextEdge(target, graph.infoGrid, graph.routingGraph);
        std::cerr << "  SAME => Route: " << sourceEdge << " to " << targetEdge << std::endl;
        const auto routeEdges = Routing::findZonePath(graph.routingGraph,
            zoneBases, zoneEdges, sourceEdge, targetEdge);

        return routeToAngle(graph, routeEdges);
	}

    // Use last Abstract Level to route.
    std::cerr << "  LAST zone" << std::endl;
    const auto& ablv = graph.abstractLevels.back();
	const auto& sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
	const auto& sourceInfo = ablv.zones[sourceZone];
	const auto& zoneBases = sourceInfo.baseNodeIdxs;
	const auto& zoneEdges = sourceInfo.baseEdgeIdxs;
	int sourceEdge = getNextEdge(source, graph.infoGrid, graph.routingGraph);

    GridType::AbstractNode abNode = ablv.abstractNodes[sourceZone];

    std::cerr << "    Use zone: " << sourceZone << " baseNode:" << abNode.baseCenterNode << " to find edge" << std::endl;
    const GridType::Point& targetPos = graph.baseNodes[abNode.baseCenterNode];
	int targetEdge = getNextEdge(targetPos, graph.infoGrid, graph.routingGraph);

	const auto routeEdges = Routing::findZonePath(graph.routingGraph,
		zoneBases, zoneEdges, sourceEdge, targetEdge);

    return routeToAngle(graph, routeEdges);
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
	std::cerr << "FROM:" << from.x << "," << from.y << " TO " << to.x << "," << to.y << "   cell:" << info.mCellWidth << "x" << info.mCaveHeight << std::endl;
	GridType::Point fromPnt = {from.x/(info.mCellWidth*8), from.y/(info.mCellHeight*8) };
	GridType::Point toPnt = {to.x/(info.mCellWidth*8), to.y/(info.mCellHeight*8) };
	std::cerr << "===GETMOVE: " << fromPnt.first << "," << fromPnt.second << " TO " << toPnt.first << "," << toPnt.second << std::endl;
//FIXME: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#if 0
	return 0;
#else
    float ang = static_cast<float>(getNextMove(graph, fromPnt, toPnt));
	std::cerr << "  RET " << ang << std::endl;
	return ang;
#endif
}

