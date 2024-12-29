#include <cassert>
#include <vector>
#include <queue>
#include <limits>

#include "ZSThinning.hpp"
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>
#include <GDDistanceMap.hpp>

#include "TGA.hpp"

using namespace godot;

std::vector<std::vector<int>> computeDistanceGrid(const std::vector<std::vector<int>>& grid, int& maxDist);
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


    int maxDist;
    std::vector<std::vector<int>> dg = computeDistanceGrid(grid, maxDist);
    std::cerr << "== DONE GRID " << dg[0].size() << "," << dg.size() << " maxD:" << maxDist << std::endl;

    const int rows = dg.size();
    const int cols = dg[0].size();
    {
    	std::vector<std::vector<int> > image;
    	for (std::vector<int>& row : dg) {
    		std::vector<int> imageRow;
    		for (int& xy : row) {
    			imageRow.push_back(xy ? 1 : 0);
    		}
    		image.push_back(imageRow);
    	}
    	std::cerr << "== MAKE THIN " << std::endl;
    	Algo::ZSThinning(image);
    	unsigned char* pPixels = new unsigned char[rows * cols *4];
    	unsigned char* pData = pPixels;
    	for (int y=rows-1; y >= 0; --y) {
    		for (int x=0; x < cols; ++x) {
    			unsigned char c = image[y][x] ? 0xff : 0x00;
    			for (int i=0; i < 4; ++i) *pData++ = c;
    		}
    	}
    	Stuff::TGA::saveToFile("THIN.tga", cols, rows, 32, pPixels);
    	delete[] pPixels;
    }

    {
    	unsigned char* pPixels = new unsigned char[rows * cols *4];
    	unsigned char* pData = pPixels;
    	for (int y=rows-1; y >= 0; --y) {
    		for (int x=0; x < cols; ++x) {
    			if (grid[y][x]) {
    				*pData++ = static_cast<unsigned char>(0x00);
    				*pData++ = static_cast<unsigned char>(0xff);
    				*pData++ = static_cast<unsigned char>(0x00);
    				*pData++ = static_cast<unsigned char>(0xff);
    			}
    			else {
    				*pData++ = static_cast<unsigned char>(0x00);
    				*pData++ = static_cast<unsigned char>(0x00);
    				*pData++ = static_cast<unsigned char>(0x00);
    				*pData++ = static_cast<unsigned char>(0x00);
    			}
    		}
    	}
    std::cerr << "== MAKE GRID " << std::endl;
    Stuff::TGA::saveToFile("GRID.tga", cols, rows, 32, pPixels);
    delete[] pPixels;

    }

    unsigned char* pPixels = new unsigned char[rows * cols *4];
    unsigned char* pData = pPixels;
    float md = maxDist;
    // Reverse y for TGA formatting
    for (int y=rows-1; y >= 0; --y) {
    //for (int y=0; y < info.mCaveHeight; ++y) {
    	for (int x=0; x < cols; ++x) {
    		std::cerr << "  " << x << "," << y << " d:" << (dg[y][x]) << std::endl;
    		*pData++ = static_cast<unsigned char>(255 * (dg[y][x]/md));
    		*pData++ = static_cast<unsigned char>(255 * (dg[y][x]/md));
    		*pData++ = static_cast<unsigned char>(255 * (dg[y][x]/md));
    		*pData++ = static_cast<unsigned char>(255 * (dg[y][x]/md));
    	}
    }
    std::cerr << "== MAKE TGA " << std::endl;
    Stuff::TGA::saveToFile("DMAP.tga", cols, rows, 32, pPixels);
    delete[] pPixels;

    {
    const int PixelSize = 4;
    const int PixelRowSize = cols * PixelSize;
    int numPixels = rows * PixelRowSize;
    unsigned char* pPixels = new unsigned char[numPixels];
    std::memset(pPixels, 0x00, numPixels);
    float md = maxDist;
    std::vector< std::pair<int,int>> pnts;
    // Reverse y for TGA formatting
    for (int y=rows-2; y > 0; --y) {
    	for (int x=1; x < cols-1; ++x) {
    		int v = dg[y][x];
			if (v >= maxDist) {
				pnts.push_back({x,y});
				std::cerr << "== STORE " << x << "," << y << std::endl;
			}
    	}
    }
    std::vector<std::pair<int,int>> next;
    while (!pnts.empty())
    {
    	for (auto [x, y] : pnts) {
    		int v = dg[y][x];
    		std::cerr << "CHK:" << x << "," << y << " " << v << std::endl;
    		if (v > 0) {
    			v -= 1;
    			unsigned char c = static_cast<unsigned char>(255 * (v/md));
    			std::cerr << "GOT " << x << "," << y << " = " << v << " (" << md << ") => " << static_cast<unsigned int>(c) << std::endl;
				for (int i=0; i < PixelSize; ++i) {
					pPixels[y*PixelRowSize + x*PixelSize +i] = 0xff; //c;
				}
    			dg[y][x] = -1;
    			if (dg[y+1][x] >= v) {next.push_back({x, y+1});
				std::cerr << "== STORE y+1: " << x << "," << y+1 << " nx:" << next.size() << std::endl;
    			}
    			if (dg[y-1][x] >= v) {next.push_back({x, y-1});
				std::cerr << "== STORE y-1: " << x << "," << y-1 << " nx:" << next.size() << std::endl;
    			}
    			if (dg[y][x-1] >= v) {next.push_back({x-1, y});
				std::cerr << "== STORE x-1: " << x-1 << "," << y << " nx:" << next.size() << std::endl;
    			}
    			if (dg[y][x+1] >= v) {next.push_back({x+1, y});
				std::cerr << "== STORE x+1: " << x+1 << "," << y << " nx:" << next.size() << std::endl;
    			}
    		}
    	}
    	std::cerr << "P:" << pnts.size() << " n:" << next.size();
    	std::swap(pnts,next);
    	next.clear();
		std::cerr << "== SWAPPED p:" << pnts.size() << " next:" << next.size()<< std::endl;
    }
    std::cerr << "== MAKE PATH " << std::endl;
    Stuff::TGA::saveToFile("PATH.tga", dg[0].size()-2, dg.size()-2, 32, pPixels);
    delete[] pPixels;

    }


/*
    {
    int max2;
    std::vector<std::vector<std::vector<int>>> dirGrid = computeDirectionalDistances(grid, max2);
    unsigned char* pPixels = new unsigned char[info.mCaveHeight * info.mCaveWidth *4];
    unsigned char* pData = pPixels;
    float md = maxDist;
    // Reverse y for TGA formatting
    for (int y=info.mCaveHeight +info.mBorderHeight*2-1; y >= 0; --y) {
    //for (int y=0; y < info.mCaveHeight +info.mBorderHeight*2; ++y) {
    	for (int x=0; x < info.mCaveWidth +info.mBorderWidth*2; ++x) {
    		int d = -1;
    		for (int i=0; i < 4; ++i) {
    			d = std::max(d,dirGrid[y][x][i]);
    			std::cerr << " " << dirGrid[y][x][i];
    		}
    		std::cerr << "|";
    		unsigned char c = static_cast<unsigned char> (255 * (d / md));
    		*pData++ = c;
    		*pData++ = c;
    		*pData++ = c;
    		*pData++ = c;
    	}
    	std::cerr << std::endl;
    }
    std::cerr << "========== MAKE TGA ================" << std::endl;
    Stuff::TGA::saveToFile("DIRMAP.tga", info.mCaveWidth+info.mBorderHeight*2, info.mCaveHeight+info.mBorderWidth*2, 32, pPixels);
    delete[] pPixels;
    std::cerr << "=================================== DONE" << std::endl;
    }
    */

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

///////////////////////////////////////////////////

// Function to compute the distance grid
std::vector<std::vector<int>> computeDistanceGrid(const std::vector<std::vector<int>>& grid, int& maxDist)
{
    int rows = grid.size();
    int cols = grid[0].size();
    maxDist = 0;

	std::cerr << "GRID ROW, COL " << cols << "x" << rows << std::endl;
	for (int y=0; y < rows; ++y) {
		for (int x=0; x < cols; ++x) {
			std::cerr << " " << grid[y][x];
		}
		std::cerr << std::endl;
	}
    // Directions for moving (up, down, left, right)
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    // Initialize the distance grid with a large value
    std::vector<std::vector<int>> distanceGrid(rows, std::vector<int>(cols, std::numeric_limits<int>::max()));

    // BFS queue to store cells (row, col)
    std::queue<std::pair<int, int>> bfsQueue;

    // Add all wall cells to the queue and initialize their distances
	std::cerr << "==== MAKE GRID " << std::endl;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c] == 1) { // Wall
                distanceGrid[r][c] = 0;
                bfsQueue.push({r, c});
            }
        }
    }

    // Perform BFS
	std::cerr << "==== Q " << bfsQueue.size() << std::endl;
    while (!bfsQueue.empty()) {
        auto [current_r, current_c] = bfsQueue.front();
        bfsQueue.pop();
        std::cerr << "  Q " << bfsQueue.size() << "  c,r " << current_c << "," << current_r << std::endl;

        // Current cell's distance
        int current_distance = distanceGrid[current_r][current_c];

        // Process neighbors
        for (const auto& [dr, dc] : directions) {
            int neighbor_r = current_r + dr;
            int neighbor_c = current_c + dc;

        std::cerr << "     moved c,r " << neighbor_c << "," << neighbor_r << " mx:" << maxDist << std::endl;

            // Ensure neighbor is within bounds and update its distance if shorter
            if (neighbor_r >= 0 && neighbor_r < rows && neighbor_c >= 0 && neighbor_c < cols) {
                if (distanceGrid[neighbor_r][neighbor_c] > current_distance + 1) {
                    distanceGrid[neighbor_r][neighbor_c] = current_distance + 1;
                    maxDist = std::max(maxDist, current_distance + 1);
                    bfsQueue.push({neighbor_r, neighbor_c});
                }
            }
        }
    }
    std::cerr << "==================" << std::endl;

    for (int cy=0; cy < rows; ++cy) {
    	for (int cx=0; cx < cols; ++cx) {
    		std::cerr << " " << distanceGrid[cy][cx];
    	}
    	std::cerr << std::endl;
    }
    return distanceGrid;
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
