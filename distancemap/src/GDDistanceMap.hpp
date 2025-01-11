#ifndef GD_DISTANCE_MAP_H
#define GD_DISTANCE_MAP_H

#include <utility>
#include <vector>
#include <map>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/tile_map_layer.hpp>
#include "GridToGraph.hpp"
#include "WallDistanceGrid.hpp"

namespace godot {

class GDDistanceMap : public Object {
	GDCLASS(GDDistanceMap, Object)

public:
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
		TileMapLayer* pTileMap;
		int mLayer;
	};

protected:
	static void _bind_methods();

	Info info;

    GridToGraph::Grid wallDistGrid;
    DistanceMap::SightGrid sightGrid;
	GridToGraph::Graph graph;

public:
	GDDistanceMap();
	~GDDistanceMap();


	GDDistanceMap* setCaveSize(godot::Vector2i sz);
	GDDistanceMap* setCellSize(godot::Vector2i sz);
	GDDistanceMap* setFloor(godot::Vector2i floor);

	void make_it(TileMapLayer* pTileMap, int layer);

	float getMove(godot::Vector2 from, godot::Vector2 to, int type);

private:
	bool isFloor(int cx, int cy, TileMapLayer* pTileMap, int layer);
	Vector2i getMapPos(int x, int y);
};

}

#endif
