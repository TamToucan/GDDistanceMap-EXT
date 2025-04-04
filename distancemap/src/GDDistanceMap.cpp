#include <vector>
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


// Helper function to compute angle
double computeAngle(GridType::Point from, GridType::Point to) {
    double dx = to.first - from.first;
    double dy = to.second - from.second;
    return std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0);
}

// Function to get the next move as an angle
double getNextMove(const GridToGraph::Graph& graph, const GridType::Point& source, const GridType::Point& target)
{
	const auto& ablv = graph.abstractLevels[0];
    const GridToGraph::GridPointInfo& sourceInfo = ablv.zoneGrid[source.second][source.first];
    const GridToGraph::GridPointInfo& targetInfo = ablv.zoneGrid[target.second][target.first];
	std::cerr << "]GETNEXTMOVE: "
			<< "FRM abE idx: " << sourceInfo.closestAbstractEdgeIdx
			<< " (abN " << ablv.abstractEdges[sourceInfo.closestAbstractEdgeIdx].from << " ->"
			<< " " << ablv.abstractEdges[sourceInfo.closestAbstractEdgeIdx].to<< ")" << std::endl
			<< " abN idx: " << sourceInfo.closestAbstractNodeIdx
			<< " (bN: " << ablv.abstractNodes[sourceInfo.closestAbstractNodeIdx].baseCenterNode
			<< " " << graph.baseNodes[ablv.abstractNodes[sourceInfo.closestAbstractNodeIdx].baseCenterNode].first
			<< "," << graph.baseNodes[ablv.abstractNodes[sourceInfo.closestAbstractNodeIdx].baseCenterNode].second << ")\n"

			<< "TO abE idx: " << targetInfo.closestAbstractEdgeIdx
			<< " (abN " << ablv.abstractEdges[targetInfo.closestAbstractEdgeIdx].from << " ->"
			<< " " << ablv.abstractEdges[targetInfo.closestAbstractEdgeIdx].to<< ")" << std::endl
			<< " abN idx: " << targetInfo.closestAbstractNodeIdx
			<< " (bN: " << ablv.abstractNodes[targetInfo.closestAbstractNodeIdx].baseCenterNode
			<< " " << graph.baseNodes[ablv.abstractNodes[targetInfo.closestAbstractNodeIdx].baseCenterNode].first
			<< "," << graph.baseNodes[ablv.abstractNodes[targetInfo.closestAbstractNodeIdx].baseCenterNode].second << ")"
			<< std::endl;

    // **1. Check if already at or adjacent to the target**
    if (std::abs(source.first - target.first) <= 1 && std::abs(source.second - target.second) <= 1) {
    	std::cerr << "1]] AT TARGET" << std::endl;
        return computeAngle(source, target);
    }

    // **2. Use Flow Field Navigation for moving between abstractNodes
    if (sourceInfo.closestAbstractNodeIdx != targetInfo.closestAbstractNodeIdx)
    {
    	const auto& flowField = graph.flowFields[sourceInfo.closestAbstractEdgeIdx];
    	auto dirCostOpt = flowField.unpack(source);
    	// Check if the source point is covered by this flow field
    	if (dirCostOpt) {
    		int dirIdx = dirCostOpt->first;
    		GridType::Point nextMove = {
    				source.first + GridType::directions[dirIdx].first,
					source.second + GridType::directions[dirIdx].second
    		};
    		std::cerr << "FLOW]] USE FLOW dirIdx:" << dirIdx << " cost:" << dirCostOpt->second << std::endl;
    		return computeAngle(source, nextMove);
    	}
    }
#if 0
    // **3. Use Base Graph if both are in the same base node**
    if (sourceInfo.closestBaseNodeIdx == targetInfo.closestBaseNodeIdx)
    {
        int sourceBaseNode = sourceInfo.closestBaseNodeIdx;
        int targetBaseNode = targetInfo.closestBaseNodeIdx;
        auto it = graph.baseGraph.find(sourceBaseNode);
        if (it != graph.baseGraph.end() && !it->second.empty()) {
        	std::cerr << "BASE]] FOUND Source Base:" << sourceBaseNode << " in BaseGraph. Check targetBase:" << targetBaseNode << std::endl;
        	const auto& neighbors = it->second;

        	int bestNextNode = -1;
        	int bestCost = std::numeric_limits<int>::max();

        	for (const auto& [neighbor, cost] : neighbors) {
        		if (neighbor == targetBaseNode) {
        			bestNextNode = neighbor;
        			break;
        		}

        		auto pathCostIt = graph.pathCostMap.find({neighbor, targetBaseNode});
        		if (pathCostIt != graph.pathCostMap.end() && pathCostIt->second < bestCost) {
        			bestCost = pathCostIt->second;
        			bestNextNode = neighbor;
        		}
        	}

        	if (bestNextNode != -1) {
        		std::cerr << "]]]] FOUND Neighbor base: " << bestNextNode << std::endl;
        		return computeAngle(source, graph.baseNodes[bestNextNode]);
        	}
        }
    }

    // 4. **Use FallbackGrid if flow field is not available**
    const auto& fallbackCell = graph.fallbackGrid[source.second][source.first];
    if (fallbackCell.nextFlowX != -1) {
    	std::cerr << "BAKFLOW]] Use fallback flow: " << fallbackCell.nextFlowX << "," << fallbackCell.nextFlowY << std::endl;
    	return computeAngle(source, {fallbackCell.nextFlowX, fallbackCell.nextFlowY});
    }

    // 5. Use graph.infoGrid to move towards the nearest baseEdge
    GridType::Point bestMove = source;
    int bestDistance = graph.infoGrid[source.second][source.first];
    std::cerr << "INFO]] Check infoGrid neighbors" << std::endl;
    for (const auto& dir : GridType::directions)
    {
    	GridType::Point neighbor = {source.first + dir.first, source.second + dir.second};
    	if (graph.infoGrid[neighbor.second][neighbor.first]&(GridToGraph::EDGE|GridToGraph::XPND)) {
    		int newDist = (graph.infoGrid[neighbor.second][neighbor.first])&0xffff;
    		std::cerr << "]]]] Check neighbor: " << neighbor.first << "," << neighbor.second << " cost:" << newDist << std::endl;
    		if (newDist < bestDistance) {
    			bestMove = neighbor;
    			bestDistance = graph.infoGrid[neighbor.second][neighbor.first];
    		}
    	}
    }
    if (bestMove != source) {
    	std::cerr << "]]]]] Use bestMove: " << bestMove.first << "," << bestMove.second << std::endl;
    	return computeAngle(source, bestMove);
    }

    // **6. Last Resort: Use Navigation Grid**
    if (sourceInfo.closestAbstractNodeIdx != -1) {
    	std::cerr << "ANGLE]] Use AngleToFromNode: " << sourceInfo.angleToToNode << std::endl;
    	return sourceInfo.angleToFromNode; // Approximate movement towards the base node
    }
#endif

    // 7. Absolute fallback: move to target
    std::cerr << "FALLBACK]] FALLBACK src->targ" << std::endl;
    return computeAngle(source, target);
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
	float ang = getNextMove(graph, fromPnt, toPnt);
	std::cerr << "  RET " << ang << std::endl;
	return ang;
#endif
}

