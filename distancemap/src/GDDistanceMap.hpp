#ifndef GD_CAVE_H
#define GD_CAVE_H

#include <utility>
#include <vector>
#include <map>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/tile_map_layer.hpp>
#include "Rect.h"

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

public:
	GDDistanceMap();
	~GDDistanceMap();


	GDDistanceMap* setFloor(godot::Vector2i floor);
	GDDistanceMap* setWall(godot::Vector2i wall);

	void make_it(TileMapLayer* pTileMap, int layer);

private:
	bool isWall(int cx, int cy, TileMapLayer* pTileMap, int layer);
	// Can't use !isWall since out of bounds returns false as well
	bool isFloor(int cx, int cy, TileMapLayer* pTileMap, int layer);
	Vector2i getMapPos(int x, int y);
};

}

#endif
