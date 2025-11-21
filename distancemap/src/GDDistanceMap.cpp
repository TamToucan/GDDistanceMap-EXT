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

#include "Debug.h"

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
    LOG_INFO("SET CAVE: " << info.mCaveWidth << "x" << info.mCaveHeight);
	return this;
}

GDDistanceMap* GDDistanceMap::setCellSize(godot::Vector2i sz) {
	info.mCellWidth = sz.width;
	info.mCellHeight = sz.height;
    LOG_INFO("SET CELL: " << info.mCellWidth << "x" << info.mCellHeight);
	return this;
}

GDDistanceMap* GDDistanceMap::setFloor(godot::Vector2i floor) {
	info.mFloor = floor;
    LOG_INFO("SET FLOOR: " << info.mFloor.x << "x" << info.mFloor.y);
	return this;
}

void GDDistanceMap::make_it(TileMapLayer* pTileMap, int layer)
{
	info.pTileMap = pTileMap;
	info.mLayer = layer;

    LOG_INFO("====================================================="
        << "##Make DistanceMap tileMap : " << info.mCaveWidth << "x" << info.mCaveHeight
        << " border:" << info.mBorderWidth << "," << info.mBorderHeight);

    LOG_DEBUG("  Copy tileMap");
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
    LOG_INFO("## makeWallDistanceGrid");
    wallDistGrid = DistanceMap::makeWallDistanceGrid(grid);

    //
    // Make a SightGrid which can return the distance to wall (N,S,E,W)
    //
    LOG_INFO("## makeSightGrid");
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
    LOG_INFO("## makeGraph");
    auto graph = GridToGraph::makeGraph(floorGrid);
    navGraph.initialize(graph, info);

}

// ===========================================================================

Vector2i GDDistanceMap::getMapPos(int x, int y) {
	return Vector2i(info.mBorderWidth+x*info.mCellWidth, info.mBorderHeight+y*info.mCellHeight);
}

/////////////////////////////////////////////////////////

static void untrack_cb(godot::Node* id, void* ctx) {
	if (ctx) {
        LOG_DEBUG("===UNTRACK id:" << id << " CTX: " << ctx);
		Router::RouteCtx* routeCtx = static_cast<Router::RouteCtx*>(ctx);
		delete routeCtx;
	}
}

float GDDistanceMap::getMove(godot::Node* id, godot::Vector2 from, godot::Vector2 to, int type) {
    Router::RouteCtx* ctx = pTracker ? pTracker->getContext<Router::RouteCtx>(id) : nullptr;

    if (!ctx) {
        LOG_DEBUG("===TRACK => CREATE CONTEXT");
		ctx = new Router::RouteCtx();
        pTracker->setContext<Router::RouteCtx>(id, ctx);
        LOG_DEBUG("===ADD UNTRACK CALLBACK");
        pTracker->set_untrack_callback(untrack_cb);
    }
	return navGraph.getMoveDirection(ctx, from, to, type);
}

