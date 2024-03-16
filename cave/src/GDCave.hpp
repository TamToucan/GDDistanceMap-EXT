#ifndef GD_CAVE_H
#define GD_CAVE_H

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/tile_map.hpp>

namespace godot {

class GDCave : public Object {
	GDCLASS(GDCave, Object)

protected:
	static void _bind_methods();

	int mCaveWidth = 2;
	int mCaveHeight = 2;
	int mBorderWidth = 1;
	int mBorderHeight = 1;
	int mCellWidth = 1;
	int mCellHeight = 1;
	int mStartCellX = 0;
	int mStartCellY = 0;

	int mOctaves = 8;

	godot::Vector2i mFloor;
	godot::Vector2i mWall;

public:
	GDCave();
	~GDCave();

	GDCave* setCaveSize(Vector2i caveSize);

	GDCave* setBorderCellSize(Vector2i border);
	GDCave* setCellSize(Vector2i cellSize);
	GDCave* setStartCell(int x, int y);

	GDCave* setFloor(godot::Vector2i floor);
	GDCave* setWall(godot::Vector2i wall);

	GDCave* setOctaves(int octaves);

	void make_it(TileMap* pTileMap, int layer, int seed);

private:
	void setCell(TileMap* pTileMap, int layer, Vector2i coords, Vector2i tile);
};

}

#endif
