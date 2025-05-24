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

#include "GDTracker.hpp"

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
	ClassDB::bind_method(D_METHOD("get_move", "node", "from", "to", "type"), &GDDistanceMap::getMove);
	ClassDB::bind_method(D_METHOD("set_tracker"), &GDDistanceMap::setTracker);
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

GridType::Point nextPoint(const GridType::Point& from, const GridType::Point& dir) {
	std::cerr << "##NEXT POINT: From: " << from.first << "," << from.second
        << " move " << dir.first << "," << dir.second << std::endl;

	return { from.first + dir.first, from.second + dir.second };
}

float computeAngle(double dx, double dy) {
	if (dx == 0 && dy == 0) {
		return 0.0; // No movement
	}
    return static_cast<float>(std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0));
}

// Helper function to compute angle
GridType::Point nextStep(const GridType::Point& from, const GridType::Point& to) {
	std::cerr << "##NEXT STEP: From: " << from.first << "," << from.second
        << " -> " << to.first << "," << to.second << std::endl;

	GridType::Point dir = { to.first - from.first, to.second - from.second };
    return nextPoint(from, dir);
}

// Forward declaration for RouteCtx if needed by getNextMove, though it's usually defined before.
struct RouteCtx;

// Function to get the next move, now with RouteCtx for state
GridType::Point getNextMove(const GridToGraph::Graph& graph, GridType::Point& source, GridType::Point& target, RouteCtx* ctx)
{
    // 1. Initial source and target validation (Boundary checks from previous version)
    int srcCell = graph.infoGrid[source.second][source.first];
    if (srcCell & GridType::WALL) {
        if (srcCell & GridType::BOUNDARY) {
            int dirIdx = srcCell & GridType::DIR_MASK;
            source.first += GridType::directions8[dirIdx].first;
            source.second += GridType::directions8[dirIdx].second;
            std::cerr << "Adjusted source from BOUNDARY: (" << source.first << "," << source.second << ") DIR: " << dirIdx << std::endl;
            srcCell = graph.infoGrid[source.second][source.first]; // Re-read cell info
        }
        if (srcCell & GridType::WALL) { // Check again after adjustment
            std::cerr << "ERROR: Source (" << source.first << "," << source.second << ") is WALL." << std::endl;
            return source; 
        }
    }

    int tgtCell = graph.infoGrid[target.second][target.first];
    if (tgtCell & GridType::WALL) {
        if (tgtCell & GridType::BOUNDARY) {
            int dirIdx = tgtCell & GridType::DIR_MASK;
            target.first += GridType::directions8[dirIdx].first;
            target.second += GridType::directions8[dirIdx].second;
            std::cerr << "Adjusted target from BOUNDARY: (" << target.first << "," << target.second << ") DIR: " << dirIdx << std::endl;
            tgtCell = graph.infoGrid[target.second][target.first]; // Re-read cell info
        }
        if (tgtCell & GridType::WALL) { // Check again after adjustment
            std::cerr << "ERROR: Target (" << target.first << "," << target.second << ") is WALL." << std::endl;
            return source; 
        }
    }

    // 2. Target Moved Check
    // Compare the current 'target' (passed from getMove) with 'ctx->to' (target for which the path was computed).
    bool targetHasMoved = manhattanDistance(target, ctx->to) > 0;
    if (targetHasMoved) {
        std::cerr << "Target has moved. Old path target: (" << ctx->to.first << "," << ctx->to.second 
                  << "), New current target: (" << target.first << "," << target.second << "). Clearing path." << std::endl;
        ctx->currentPath.clear();
        ctx->currentPathIndex = 0;
        ctx->to = target; // Update path target to the new current target
    }

    // 3. Path Initialization or Re-computation
    // Recompute if path is empty, index is out of bounds, OR if the target has moved (forcing replan for new target)
    if (ctx->currentPath.empty() || ctx->currentPathIndex >= ctx->currentPath.size() || targetHasMoved) {
        std::cerr << "Path empty, index out of bounds, or target moved. Source: (" << source.first << "," << source.second
                  << ") Current Target: (" << target.first << "). Recomputing." << std::endl;
        ctx->currentPath.clear();
        ctx->currentPathIndex = 0;
        // Ensure ctx->to reflects the target we are NOW pathfinding for.
        // This is crucial if targetHasMoved was false, but path was empty for other reasons.
        ctx->to = target; 


        int sourceEdgeIdx = graph.routingGraph.edgeGrid.get(source.first, source.second) & 0x7FFF;
        int targetEdgeIdx = graph.routingGraph.edgeGrid.get(target.first, target.second) & 0x7FFF;
        
        std::cerr << "  Source Point: (" << source.first << "," << source.second << ") -> Source Edge Idx: " << sourceEdgeIdx << std::endl;
        std::cerr << "  Target Point: (" << target.first << "," << target.second << ") -> Target Edge Idx: " << targetEdgeIdx << std::endl;

        // Validate sourceEdgeIdx
        if (sourceEdgeIdx == 0x7FFF || sourceEdgeIdx >= graph.routingGraph.edgeFromTos.size()) {
            std::cerr << "  ERROR: Invalid sourceEdgeIdx: " << sourceEdgeIdx 
                      << " (max valid: " << (graph.routingGraph.edgeFromTos.empty() ? -1 : (int)graph.routingGraph.edgeFromTos.size() - 1) 
                      << "). Cannot pathfind." << std::endl;
            return source; 
        }
        // Validate targetEdgeIdx
        if (targetEdgeIdx == 0x7FFF || targetEdgeIdx >= graph.routingGraph.edgeFromTos.size()) {
            std::cerr << "  ERROR: Invalid targetEdgeIdx: " << targetEdgeIdx 
                      << " (max valid: " << (graph.routingGraph.edgeFromTos.empty() ? -1 : (int)graph.routingGraph.edgeFromTos.size() - 1) 
                      << "). Cannot pathfind." << std::endl;
            return source; 
        }

        std::vector<int> zoneBases;
        for (size_t i = 0; i < graph.routingGraph.nodePoints.size(); ++i) {
            zoneBases.push_back(i);
        }
        std::vector<int> zoneEdges;
        for (size_t i = 0; i < graph.routingGraph.edgeFromTos.size(); ++i) {
            zoneEdges.push_back(i);
        }
        
        // Ensure source/target are not walls before pathfinding
        if ((graph.infoGrid[source.second][source.first] & GridType::WALL) || 
            (graph.infoGrid[target.second][target.first] & GridType::WALL)) { // 'target' here is the current frame's target
            std::cerr << "  Cannot pathfind: source or current target is a wall after adjustments." << std::endl;
            return source;
        }

        ctx->currentPath = Routing::findZonePath(graph.routingGraph, zoneBases, zoneEdges, sourceEdgeIdx, targetEdgeIdx);
        
        // Associate the computed path with the target it was computed for.
        // This is important because 'target' (the parameter) might change frame to frame.
        // 'ctx->to' should always reflect the target of 'ctx->currentPath'.
        if (!ctx->currentPath.empty()) {
             ctx->to = target; // Path is for the 'target' it was just computed with.
             std::cerr << "  Path found with " << ctx->currentPath.size() << " points for target (" 
                       << ctx->to.first << "," << ctx->to.second << "). First point: ("
                       << ctx->currentPath[0].first << "," << ctx->currentPath[0].second << ")" << std::endl;

            // 3.1 Path Validation: Check if any point on the new path is a wall
            bool newPathIsWallFree = true;
            for (const auto& pathPoint : ctx->currentPath) {
                if (!isValidGridPoint(graph.infoGrid, pathPoint) || (graph.infoGrid[pathPoint.second][pathPoint.first] & GridType::WALL)) {
                    std::cerr << "  New path validation FAILED: point (" << pathPoint.first << "," << pathPoint.second
                              << ") is a wall or invalid." << std::endl;
                    newPathIsWallFree = false;
                    break;
                }
            }
            if (!newPathIsWallFree) {
                ctx->currentPath.clear();
                ctx->currentPathIndex = 0;
                // ctx->to remains as the 'target' we attempted to path to, so next frame, if target hasn't changed, we might retry.
                std::cerr << "  New path invalidated due to containing a wall." << std::endl;
                return source; // Stay put
            }
            std::cerr << "  New path validated successfully (all points are floor)." << std::endl;
        } else {
            std::cerr << "  No path found from edge " << sourceEdgeIdx << " to edge " << targetEdgeIdx 
                      << " for target (" << target.first << "," << target.second << ")." << std::endl;
            // If no path, ctx->to still holds the target we failed to path to.
            // If target changes next frame, targetHasMoved will be true.
            // If target doesn't change, we'll retry pathing to same ctx->to.
            return source; 
        }
    }

    // 4. Path Following
    if (!ctx->currentPath.empty() && ctx->currentPathIndex < ctx->currentPath.size()) {
        GridType::Point nextNodeOnPath = ctx->currentPath[ctx->currentPathIndex];
        std::cerr << "Following path. Index: " << ctx->currentPathIndex << ", Attempting to move to: ("
                  << nextNodeOnPath.first << "," << nextNodeOnPath.second << ")" << std::endl;

        if (isValidGridPoint(graph.infoGrid, nextNodeOnPath) &&
            (graph.infoGrid[nextNodeOnPath.second][nextNodeOnPath.first] & GridType::WALL)) {
            std::cerr << "  WARN: Next point on path (" << nextNodeOnPath.first << "," << nextNodeOnPath.second
                      << ") is a wall. Attempting local adjustment from source (" 
                      << source.first << "," << source.second << ")." << std::endl;

            GridType::Point bestNeighbor = source;
            int minDistanceToTarget = manhattanDistance(source, target); // Current distance if no move
            bool foundAdjustment = false;

            for (const auto& dir : GridType::directions8) {
                GridType::Point neighbor = {source.first + dir.first, source.second + dir.second};
                if (isValidGridPoint(graph.infoGrid, neighbor) &&
                    !(graph.infoGrid[neighbor.second][neighbor.first] & GridType::WALL)) {
                    
                    int distToTarget = manhattanDistance(neighbor, target);
                    // Prefer neighbors that make progress towards the target
                    if (distToTarget < minDistanceToTarget) {
                        minDistanceToTarget = distToTarget;
                        bestNeighbor = neighbor;
                        foundAdjustment = true;
                    } else if (!foundAdjustment && distToTarget == minDistanceToTarget) { 
                        // If no better option found yet, take one that's same distance
                        // (could be a side step to get unstuck)
                        bestNeighbor = neighbor;
                        foundAdjustment = true;
                    }
                }
            }

            if (foundAdjustment && (bestNeighbor.first != source.first || bestNeighbor.second != source.second) ) {
                std::cerr << "  Local adjustment: Moving to neighbor (" << bestNeighbor.first << "," << bestNeighbor.second 
                          << ") instead of blocked path point. Path preserved." << std::endl;
                // Path index is NOT incremented; we took a detour.
                // The next call will re-evaluate from bestNeighbor.
                return bestNeighbor;
            } else {
                std::cerr << "  No suitable non-wall neighbor found for local adjustment or no progress possible. Clearing path." << std::endl;
                ctx->currentPath.clear();
                ctx->currentPathIndex = 0;
                return source; // Stay put, force replan
            }
        } else if (!isValidGridPoint(graph.infoGrid, nextNodeOnPath)) {
             std::cerr << "  ERROR: Next point on path (" << nextNodeOnPath.first << "," << nextNodeOnPath.second
                      << ") is outside grid boundaries. Clearing path." << std::endl;
            ctx->currentPath.clear();
            ctx->currentPathIndex = 0;
            return source; 
        }


        // If nextNodeOnPath is valid and not a wall
        ctx->currentPathIndex++;
        return nextNodeOnPath;
    }

    std::cerr << "Path ended or invalid state. Current source: (" << source.first << "," << source.second << ")" << std::endl;
    return source; // Fallback
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
struct RouteCtx {
	GridType::Point from;
	GridType::Point to;       // Target point for the current path
	GridType::Point next;     // Next immediate step to take
	int type;
    float curDir;
    GridType::Path currentPath; // Stores the list of points (cell coordinates) for the current path
    int currentPathIndex = 0;   // Index of the next point to take in currentPath
};

static void untrack_cb(godot::Node* id, void* ctx) {
	if (ctx) {
		std::cerr << "===UNTRACK id:" << id << " CTX: " << ctx << std::endl;
		RouteCtx* routeCtx = static_cast<RouteCtx*>(ctx);
		delete routeCtx;
	}
}

float GDDistanceMap::getMove(godot::Node* id, godot::Vector2 from, godot::Vector2 to, int type) {
	std::cerr << "********** FROM:" << from.x << "," << from.y << " TO " << to.x << "," << to.y << "   cell:" << info.mCellWidth << "x" << info.mCaveHeight << std::endl;
	GridType::Point fromPnt = {from.x/(info.mCellWidth*8), from.y/(info.mCellHeight*8) };
	GridType::Point toPnt = {to.x/(info.mCellWidth*8), to.y/(info.mCellHeight*8) };

    RouteCtx* ctx = pTracker ? pTracker->getContext<RouteCtx>(id) : nullptr;
    std::cerr << "tracker:" << pTracker << " ID:" << id << " CTX = " << ctx << std::endl;

    if (!ctx) {
		std::cerr << "===CREATE CONTEXT for ID: " << id << std::endl;
		ctx = new RouteCtx();
        ctx->from = fromPnt;
        ctx->to = toPnt;     // Initialize to the initial target
        ctx->next = fromPnt; // Start by assuming 'next' is current position
        ctx->type = type;
        ctx->currentPath.clear();
        ctx->currentPathIndex = 0;
        pTracker->setContext<RouteCtx>(id, ctx);
		std::cerr << "===ADD CALLBACK for new context" << std::endl;
        pTracker->set_untrack_callback(untrack_cb);
    }

    // Always update context with current dynamic values from this frame
    ctx->from = fromPnt;
    // REMOVED: ctx->to = toPnt; 
    // 'ctx->to' now specifically means the target for which 'ctx->currentPath' was generated.
    // 'toPnt' (current frame's target) is passed directly to getNextMove.
    ctx->type = type;


    // Caching Logic:
    // If the entity is not at the `ctx->next` point (which was determined in a *previous* call to getMove/getNextMove),
    // it means the entity is still in transit towards that `ctx->next`. So, reuse `ctx->curDir`.
    // `getNextMove` will handle path invalidation if `toPnt` (the ultimate target) changes significantly.
    if (ctx->type == type && (fromPnt.first != ctx->next.first || fromPnt.second != ctx->next.second)) {
        std::cerr << "Entity at (" << fromPnt.first << "," << fromPnt.second
                  << ") has not reached previous ctx->next (" << ctx->next.first << "," << ctx->next.second
                  << "). Reusing curDir: " << ctx->curDir << " to reach it." << std::endl;
        return ctx->curDir;
    }
    // If we are here, it means fromPnt == ctx->next (or it's the first call for a new context).
    // We need to determine the *new* ctx->next.

	std::cerr << "===GETMOVE: Current (" << fromPnt.first << "," << fromPnt.second 
              << ") Target (" << toPnt.first << "," << toPnt.second 
              << ") PathIdx: " << ctx->currentPathIndex << " PathSize: " 
              << (ctx->currentPath.empty() ? 0 : ctx->currentPath.size()) << std::endl;
    
    // Pass the current `toPnt` as the target for `getNextMove`.
    // `ctx->to` was updated above and `getNextMove` will use that for its internal logic.
    ctx->next = getNextMove(graph, fromPnt, toPnt, ctx);

    // The angle should be from the current position (ctx->from) to the newly determined next step (ctx->next)
	ctx->curDir = computeAngle(ctx->next.first - ctx->from.first, ctx->next.second - ctx->from.second);
	std::cerr << "##ANG from (" << ctx->from.first << "," << ctx->from.second << ") to ctx->next: ("
              << ctx->next.first <<"," << ctx->next.second << ") = " << ctx->curDir << std::endl;
	return ctx->curDir;
}

