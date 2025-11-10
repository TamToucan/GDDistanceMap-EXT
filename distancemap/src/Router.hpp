#pragma once

#include <vector>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/classes/tile_map_layer.hpp>

#include "GridTypes.hpp"
#include "GridToGraph.hpp"
#include "GDDistanceMapApi.h"


namespace Router {

struct RouteCtx {
    int type = -1;
    float curDir;
    GridType::Point from;
	GridType::Point to;
    GridType::Point next;
    std::vector<int> routeNodes;

    // Limit number of times can keep reusing previous direction
    // before re-routing
    int reuseInit = 0; 
    int reuseCnt = reuseInit;
    bool didReuse = false;

    // Cache for each type
    int routeSrcNodeIdx = -1;
    int routeTgtNodeIdx = -1;

    int routeSrcNodeIdx2 = -1;
    int routeTgtEdgeIdx  = -1;

    int routeSrcEdgeIdx  = -1;
    int routeTgtNodeIdx2 = -1;

    int routeSrcEdgeIdx2 = -1;
    int routeTgtEdgeIdx2 = -1;

    enum class RouteType { None, NodeToNode, NodeToEdge, EdgeToNode, EdgeToEdge };
    RouteType lastRouteType = RouteType::None;
};


struct Info {
	int mCaveWidth = 2;
	int mCaveHeight = 2;
	int mBorderWidth = 1;
	int mBorderHeight = 1;
	int mCellWidth = 1;
	int mCellHeight = 1;
	int mStartCellX = 0;
	int mStartCellY = 0;
	godot::Vector2i mFloor;
	godot::Vector2i mWall;
	godot::TileMapLayer* pTileMap;
	int mLayer;
};

GDDISTANCE_MAP_API float getAngle( const GridToGraph::Graph& graph, const Info& info, RouteCtx* ctx, godot::Vector2 from, godot::Vector2 to, int type);

} // namespace Router
