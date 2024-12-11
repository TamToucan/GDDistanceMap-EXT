#ifndef GD_CAVE_H
#define GD_CAVE_H

#include <utility>
#include <vector>
#include <map>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/tile_map.hpp>
#include "DisjointSets.h"
#include "Rect.h"

namespace godot {

struct Vector2Hash {
    size_t operator()(const Vector2i& v) const {
        // Combine hashes of x and y (simple example, can be more complex if needed).
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1);
    }

};

struct Vector2Equal {
	bool operator()(const Vector2i& lhs, const Vector2i& rhs) const {
		return lhs.x == rhs.x && lhs.y == rhs.y;
	}
};

class GDCave : public Object {
	GDCLASS(GDCave, Object)

	using Vector2iIntMap = std::unordered_map<Vector2i, int, godot::Vector2Hash, godot::Vector2Equal>;
	using IntVectorOfVector2iMap = std::unordered_map<int, std::vector<Vector2i>> ;

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
		TileMap* pTileMap;
		int mLayer;
	};

protected:
	static void _bind_methods();

	Info info;

	int mOctaves = 8;
	bool mPerlin = false;
	// If Perlin is false this is the % (0..1) that are walls for random init
	float mWallChance = 0;
	float mFreq = 1;
	float mAmp = 1;
	godot::Array mGenerations;

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
	GDCave* setPerlin(bool usePerlin);
	// 0.5 = 50% chance of wall in initial setup (if not perlin)
	GDCave* setWallChance(float wallChance);
	GDCave* setFreq(float freq);
	GDCave* setAmp(float amp);
	GDCave* setGenerations(const godot::Array& gens);

	void make_it(TileMap* pTileMap, int layer, int seed);

private:
	struct BorderWall;

	void fixup(TileMap* pTileMap, int layer);

	std::pair< Vector2iIntMap, IntVectorOfVector2iMap > findRooms(TileMap* pTileMap, int layer);

	void joinRooms(TileMap* pTileMap, int layer, std::pair<Vector2iIntMap, IntVectorOfVector2iMap> floorMaps);
	std::vector<BorderWall> detectBorderWalls(TileMap* pTileMap, int layer, std::pair<Vector2iIntMap, IntVectorOfVector2iMap> floorMaps);
	std::vector<BorderWall> findMST_Kruskal(std::vector<GDCave::BorderWall>& borderWalls, std::vector<int> roomIds);

private:
	void setCell(TileMap* pTileMap, int layer, Vector2i coords, Vector2i tile);
	Vector2i getMapPos(int x, int y) {
		return Vector2i(info.mBorderWidth+x*info.mCellWidth, info.mBorderHeight+y*info.mCellHeight);
	}
	static int getIdForXY(int x, int y) {
		return y << 16 | x;
	}
	bool isWall(int cx, int cy, TileMap* pTileMap, int layer);
	// Can't use !isWall since out of bounds returns false as well
	bool isFloor(int cx, int cy, TileMap* pTileMap, int layer);
};

}

#endif
