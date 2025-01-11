#include <vector>

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>
#include <GDDistanceMap.hpp>
#include <unordered_map>

#include "WallDistanceGrid.hpp"
#include "GridToGraph.hpp"

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
	std::cerr << "CAVE: " << info.mCaveWidth << "x" << info.mCaveHeight << std::endl;
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
    GridToGraph::Grid floorGrid;
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

float GDDistanceMap::getMove(godot::Vector2 from, godot::Vector2 to, int type)
{
	std::cerr << "==GETMOVE: " << from.x << "," << from.x << std::endl;
	// *8 since base tile is 8x8
	int fx = from.x / (info.mCellWidth * 8);
	int fy = from.y / (info.mCellWidth * 8);
	int tx = to.x / (info.mCellWidth * 8);
	int ty = to.y / (info.mCellWidth * 8);
	std::cerr << "  FROM: " << fx << "," << fy << " TO: " << tx << "," << ty << std::endl;

	/// Get Nav info for the from (source) and to (dest)
	const auto& fromInfo = graph.navGrid[fy][fx];
	const auto& toInfo = graph.navGrid[ty][tx];

	// Get the nodes at the end of the closest edge to the from/source location
	const int closeFromFrom = graph.baseEdges[fromInfo.closestBaseEdgeIdx].from;
	const int closeFromTo = graph.baseEdges[fromInfo.closestBaseEdgeIdx].to;

	// Get the nodes at the end of the closest edge to the to/dest location
	const int closeToFrom = graph.baseEdges[toInfo.closestBaseEdgeIdx].from;
	const int closeToTo = graph.baseEdges[toInfo.closestBaseEdgeIdx].to;

	// Get the Path Cost (length) from the 2 nodes at the source edge to the 2 nodes of dest edge
	auto find_fftf = graph.pathCostMap.find({closeFromFrom, closeToFrom});
	auto find_fftt = graph.pathCostMap.find({closeFromFrom, closeToTo});
	auto find_fttf = graph.pathCostMap.find({closeFromTo, closeToFrom});
	auto find_fttt = graph.pathCostMap.find({closeFromTo, closeToTo});

	// Get an initial dir just based on 50/50 moving from source to edgeFrom of edgeTo nodes
	float dir = (static_cast<int>(from.x)&0x1) ? fromInfo.angleToFromNode : fromInfo.angleToFromNode;

	// If for some reason (shouldn't happen)
	if ( (find_fftf == graph.pathCostMap.end())
		|| (find_fftt == graph.pathCostMap.end())
		|| (find_fttf == graph.pathCostMap.end())
		|| (find_fttt == graph.pathCostMap.end()) ) {
		std::cerr << "ERROR: Dont have 4 paths from nodes srcFF:" << closeFromFrom << " srcTF" << closeToFrom
				<< "  dstTF: " << closeToFrom << " dstTT: " << closeToTo << std::endl;
		return dir;
	}
    int minCost = std::numeric_limits<int>::max();
	if (find_fftf->second < minCost) {
		minCost = find_fftf->second;
		dir = fromInfo.angleToFromNode;
	}
	if (find_fftt->second < minCost) {
		minCost = find_fftt->second;
		dir = fromInfo.angleToFromNode;
	}
	if (find_fttf->second < minCost) {
		minCost = find_fttf->second;
		dir = fromInfo.angleToToNode;
	}
	if (find_fttt->second < minCost) {
		minCost = find_fttt->second;
		dir = fromInfo.angleToToNode;
	}

	return dir;
}
