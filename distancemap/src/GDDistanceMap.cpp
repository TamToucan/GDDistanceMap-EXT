#include <cassert>
#include <vector>

#include "WallDistanceGrid.hpp"
#include "GridToGraph.hpp"
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

std::vector<std::vector<std::vector<int>>> computeDirectionalDistances(const std::vector<std::vector<int>>& grid, int& maxDist);

void GDDistanceMap::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_floor", "floorTileCoords"), &GDDistanceMap::setFloor);
	ClassDB::bind_method(D_METHOD("set_cave_size", "caveSize"), &GDDistanceMap::setCaveSize);
	ClassDB::bind_method(D_METHOD("set_cell_size", "cellSize"), &GDDistanceMap::setCellSize);
	ClassDB::bind_method(D_METHOD("make_distance_map", "pTileMap", "layer"), &GDDistanceMap::make_it);
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






int countNeighbors(const std::vector<std::vector<int> >& image, int x, int y)
{
	return image[y-1][x-1]
		+ image[y-1][x]
		+ image[y-1][x+1]
		+ image[y][x-1]
		+ image[y][x+1]
		+ image[y+1][x-1]
		+ image[y+1][x]
		+ image[y+1][x+1];
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


    std::vector<std::vector<int>> wallDistGrid = DistanceMap::makeWallDistanceGrid(grid);

    GridToGraph::Grid floorGrid;
    for (const std::vector<int>& row : wallDistGrid) {
    	std::vector<int> floorRow;
    	for (int xy : row) {
    		floorRow.push_back(xy ? 1 : 0);  // xy != 0 => floor => set
    	}
    	floorGrid.push_back(floorRow);
    }

    GridToGraph::makeGraph(floorGrid);
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

// Function to compute directional distances
std::vector<std::vector<std::vector<int>>> computeDirectionalDistances(const std::vector<std::vector<int>>& grid, int& maxDist) {
    int rows = grid.size();
    int cols = grid[0].size();


    // Initialize a 3D vector to store distances in 4 directions for each cell
    // distance[r][c][0] = North
    // distance[r][c][1] = East
    // distance[r][c][2] = South
    // distance[r][c][3] = West
    std::vector<std::vector<std::vector<int>>> distance(rows, std::vector<std::vector<int>>(cols, std::vector<int>(4, -1)));

    maxDist = 0;

    // 1. North Sweep
    for (int c = 0; c < cols; ++c) {
        for (int r = 0; r < rows; ++r) {
            if (grid[r][c]) {
                distance[r][c][0] = 0; // Wall itself
            } else if (r > 0) {
                distance[r][c][0] = distance[r - 1][c][0] + 1; // Distance from the cell above
                maxDist = std::max(distance[r][c][0], maxDist);
            } else {
                distance[r][c][0] = -1; // No wall in this direction
            }
        }
    }

    // 2. East Sweep
    for (int r = 0; r < rows; ++r) {
        for (int c = cols - 1; c >= 0; --c) {
            if (grid[r][c]) {
                distance[r][c][1] = 0; // Wall itself
            } else if (c < cols - 1) {
                distance[r][c][1] = distance[r][c + 1][1] + 1; // Distance from the cell to the right
                maxDist = std::max(distance[r][c][0], maxDist);
            } else {
                distance[r][c][1] = -1; // No wall in this direction
            }
        }
    }

    // 3. South Sweep
    for (int c = 0; c < cols; ++c) {
        for (int r = rows - 1; r >= 0; --r) {
            if (grid[r][c]) {
                distance[r][c][2] = 0; // Wall itself
            } else if (r < rows - 1) {
                distance[r][c][2] = distance[r + 1][c][2] + 1; // Distance from the cell below
                maxDist = std::max(distance[r][c][0], maxDist);
            } else {
                distance[r][c][2] = -1; // No wall in this direction
            }
        }
    }

    // 4. West Sweep
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c]) {
                distance[r][c][3] = 0; // Wall itself
            } else if (c > 0) {
                distance[r][c][3] = distance[r][c - 1][3] + 1; // Distance from the cell to the left
                maxDist = std::max(distance[r][c][0], maxDist);
            } else {
                distance[r][c][3] = -1; // No wall in this direction
            }
        }
    }

    return distance;
}
