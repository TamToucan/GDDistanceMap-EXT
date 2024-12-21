#include <cassert>
#include <vector>
#include <queue>
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

using namespace godot;

void GDDistanceMap::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_floor", "floorTileCoords"), &GDDistanceMap::setFloor);
	ClassDB::bind_method(D_METHOD("set_wall", "wallTileCoords"), &GDDistanceMap::setWall);
	ClassDB::bind_method(D_METHOD("make_dstance_map", "pTileMap", "layer", "seed"), &GDDistanceMap::make_it);
}

GDDistanceMap::GDDistanceMap() {
	info.mFloor = Vector2i(0,0);
	info.mWall = Vector2i(0,1);
}

GDDistanceMap::~GDDistanceMap() {
}


GDDistanceMap* GDDistanceMap::setFloor(godot::Vector2i floor) {
	info.mFloor = floor;
	return this;
}

GDDistanceMap* GDDistanceMap::setWall(godot::Vector2i wall) {
	info.mWall = wall;
	return this;
}

void GDDistanceMap::make_it(TileMapLayer* pTileMap, int layer)
{
	info.pTileMap = pTileMap;
	info.mLayer = layer;
}

// ===========================================================================

bool GDDistanceMap::isWall(int cx, int cy, TileMapLayer* pTileMap, int layer) {
	if (cx >= 0 && cx < info.mCaveWidth && cy >= 0 && cy < info.mCaveHeight) {
		Vector2i coords = getMapPos(cx,cy);
		return (pTileMap->get_cell_atlas_coords(coords) == info.mWall);
	}
	return false;
}

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

///////////////////////////////////////////////////

// Function to compute the distance grid
std::vector<std::vector<int>> computeDistanceGrid(const std::vector<std::vector<int>>& grid) {
    int rows = grid.size();
    int cols = grid[0].size();

    // Directions for moving (up, down, left, right)
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    // Initialize the distance grid with a large value
    std::vector<std::vector<int>> distanceGrid(rows, std::vector<int>(cols, std::numeric_limits<int>::max()));

    // BFS queue to store cells (row, col)
    std::queue<std::pair<int, int>> bfsQueue;

    // Add all wall cells to the queue and initialize their distances
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c] == 0) { // Wall
                distanceGrid[r][c] = 0;
                bfsQueue.push({r, c});
            }
        }
    }

    // Perform BFS
    while (!bfsQueue.empty()) {
        auto [current_r, current_c] = bfsQueue.front();
        bfsQueue.pop();

        // Current cell's distance
        int current_distance = distanceGrid[current_r][current_c];

        // Process neighbors
        for (const auto& [dr, dc] : directions) {
            int neighbor_r = current_r + dr;
            int neighbor_c = current_c + dc;

            // Ensure neighbor is within bounds and update its distance if shorter
            if (neighbor_r >= 0 && neighbor_r < rows && neighbor_c >= 0 && neighbor_c < cols) {
                if (distanceGrid[neighbor_r][neighbor_c] > current_distance + 1) {
                    distanceGrid[neighbor_r][neighbor_c] = current_distance + 1;
                    bfsQueue.push({neighbor_r, neighbor_c});
                }
            }
        }
    }

    return distanceGrid;
}
