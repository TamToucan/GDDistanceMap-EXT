#include <cassert>
#include <sstream>
#include <map>
#include <set>
#include <vector>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>
#include <GDCave.hpp>
#include "RandSimple.h"
#include "DJSets.h"
#include "DisjointSets.h"
#include "RogueCave.hpp"
#include "CaveSmoother.h"
#include "DeleteIt.h"
#include "PerlinNoise.h"
#include "SimplexNoise.h"
#include "noise1234.h"
#include "Rect.h"

using namespace godot;

void GDCave::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cave_size", "caveSize"), &GDCave::setCaveSize);
	ClassDB::bind_method(D_METHOD("set_border_cell_size", "cells"), &GDCave::setBorderCellSize);
	ClassDB::bind_method(D_METHOD("set_cell_size", "cells"), &GDCave::setCellSize);
	ClassDB::bind_method(D_METHOD("set_start_cell", "x", "y"), &GDCave::setStartCell);
	ClassDB::bind_method(D_METHOD("set_floor", "floorTileCoords"), &GDCave::setFloor);
	ClassDB::bind_method(D_METHOD("set_wall", "wallTileCoords"), &GDCave::setWall);
	ClassDB::bind_method(D_METHOD("set_octaves", "octaves"), &GDCave::setOctaves);
	ClassDB::bind_method(D_METHOD("set_perlin", "usePerlin"), &GDCave::setPerlin);
	ClassDB::bind_method(D_METHOD("set_wall_chance", "wallChance"), &GDCave::setWallChance);
	ClassDB::bind_method(D_METHOD("set_freq", "freq"), &GDCave::setFreq);
	ClassDB::bind_method(D_METHOD("set_amp", "amp"), &GDCave::setAmp);
	ClassDB::bind_method(D_METHOD("set_generations", "gens"), &GDCave::setGenerations);
	ClassDB::bind_method(D_METHOD("make_cave", "pTileMap", "layer", "seed"), &GDCave::make_it);
}

GDCave::GDCave() {
	info.mFloor = Vector2i(0,0);
	info.mWall = Vector2i(0,1);

   Algo::DJSets sets;
   sets.AddElements(10);
}

GDCave::~GDCave() {
}


GDCave* GDCave::setCaveSize(Vector2i caveSize) {
	info.mCaveWidth = caveSize.x;
	info.mCaveHeight = caveSize.y;
	return this;
}

GDCave* GDCave::setBorderCellSize(Vector2i cells) {
	info.mBorderWidth = cells.x;
	info.mBorderHeight = cells.y;
	return this;
}

GDCave* GDCave::setCellSize(Vector2i cells) {
	info.mCellWidth = cells.x;
	info.mCellHeight = cells.y;
	return this;
}

GDCave* GDCave::setStartCell(int x, int y) {
	info.mStartCellX = x;
	info.mStartCellY = y;
	return this;
}

GDCave* GDCave::setFloor(godot::Vector2i floor) {
	info.mFloor = floor;
	return this;
}

GDCave* GDCave::setWall(godot::Vector2i wall) {
	info.mWall = wall;
	return this;
}

GDCave* GDCave::setOctaves(int octaves) {
	mOctaves = octaves;
	return this;
}

GDCave* GDCave::setWallChance(float wallChance) {
	mWallChance = wallChance;
	return this;
}

GDCave* GDCave::setPerlin(bool usePerlin) {
	mPerlin = usePerlin;
	return this;
}

GDCave* GDCave::setFreq(float freq) {
	mFreq = freq;
	return this;
}

GDCave* GDCave::setAmp(float amp) {
	mAmp = amp;
	return this;
}

GDCave* GDCave::setGenerations(const godot::Array& gens) {
	// Ensure the input is valid
    Array result;
    for (int i = 0; i < gens.size(); ++i) {
    	if (gens[i].get_type() == Variant::PACKED_INT32_ARRAY) {
    		PackedInt32Array ca = gens[i];
    		// bMin, bMax, sMin, sMax, reps
    		if (ca.size() == 9) {
    			mGenerations.append(ca);
    		}
        }
    }
    return this;
}

void GDCave::make_it(TileMap* pTileMap, int layer, int seed)
{
	RNG::RandSimple simple(seed);
	// So don't need to worry about -1,-1 adjust from 0,0 pos when counting neighbors
	if (info.mCellWidth > info.mBorderWidth || info.mCellHeight > info.mBorderHeight) {
		std::stringstream ss;
		ss << "Cell Width&Height must be <= Border Width&Height";
		ERR_PRINT(ss.str().c_str());
		return;
	}
	//
	// Make the border
	// - Top/Bottom
	//
	for (int cy=0; cy < info.mBorderHeight; ++cy) {
		for (int cx=0; cx < 2*info.mBorderWidth +(info.mCaveWidth*info.mCellWidth); ++cx) {
			Vector2i coordsTop(cx, cy);
			Vector2i coordsBtm(cx, cy+info.mCaveHeight*info.mCellHeight+info.mBorderHeight);
			pTileMap->set_cell(layer,coordsTop,0,info.mWall,0);
			pTileMap->set_cell(layer,coordsBtm,0,info.mWall,0);
		}
	}
	// - Left/Right
	for (int cx=0; cx < info.mBorderWidth; ++cx) {
		for (int cy=0; cy < 2*info.mBorderHeight +(info.mCaveHeight*info.mCellHeight); ++cy) {
			Vector2i coordsLft(cx, cy);
			Vector2i coordsRgt(cx+info.mCaveWidth*info.mCellWidth+info.mBorderWidth, cy);
			pTileMap->set_cell(layer,coordsLft,0,info.mWall,0);
			pTileMap->set_cell(layer,coordsRgt,0,info.mWall,0);
		}
	}

	//
	// Fill with random or perlin
	//
	double W = info.mCaveWidth-1 +mAmp;
	double H = info.mCaveHeight-1 +mAmp;
	double (*pf)(double, double, int) = mPerlin ? &Algo::getSNoise2 : &Algo::getNoise2;
	for (int cy=0; cy < info.mCaveHeight; ++cy) {
		for (int cx=0; cx < info.mCaveWidth; ++cx) {
			Vector2i coords = getMapPos(cx,cy);
			double x = cx/W *mFreq;
			double y = cy/H *mFreq;
			double n1 = mPerlin ? (*pf)(x,y,mOctaves) : Math::absf(simple.getFloat())-mWallChance;
			if (n1 < 0) {
				setCell(pTileMap, layer,coords,info.mWall);
			}
			else {
				setCell(pTileMap, layer,coords,info.mFloor);
			}
		}
	}


	//
	// Cellular automata
	//
	if (! mGenerations.is_empty()) {
		// - Setup grid for RogueCave
		PCG::RogueCave cave(info.mCaveWidth, info.mCaveHeight);
		std::vector<std::vector<int>>& gridIn = cave.getGrid();
		for (int cy=0; cy < info.mCaveHeight; ++cy) {
			for (int cx=0; cx < info.mCaveWidth; ++cx) {
				Vector2i coords = getMapPos(cx,cy);
				gridIn[cy][cx] = pTileMap->get_cell_atlas_coords(layer,coords,false) == info.mWall
						? PCG::RogueCave::TILE_WALL
						: PCG::RogueCave::TILE_FLOOR;
			}
		}

		// - Run Cellular Automata to get map
		for (int cidx=0; cidx < mGenerations.size(); ++cidx) {
			PackedInt32Array ca = mGenerations[cidx];
			Util::IntRange b3(ca[0], ca[1]);
			Util::IntRange b5(ca[2], ca[3]);
			Util::IntRange s3(ca[4], ca[5]);
			Util::IntRange s5(ca[6], ca[7]);
			cave.addGeneration(b3, b5, s3, s5, ca[8]);
		}
		std::vector<std::vector<int>>& gridOut = cave.generate();

		// - Copy RogueCave back to map
		std::vector<Vector2i> walls;
		std::vector<Vector2i> floors;
		for(int cy=0; cy<info.mCaveHeight; ++cy) {
			for(int cx=0; cx<info.mCaveWidth; ++cx) {
				Vector2i coords = getMapPos(cx,cy);
				if (gridOut[cy][cx] == PCG::RogueCave::TILE_WALL) {
					walls.push_back(coords);
				}
				else {
					floors.push_back(coords);
				}
			}
		}
		for (Vector2i corner : walls) {
			setCell(pTileMap, layer, corner, info.mWall);
		}
		for (Vector2i corner : floors) {
			setCell(pTileMap, layer, corner, info.mFloor);
		}
	}

	//
	// Remove cells just touching on diagonal etc
	//
	fixup(pTileMap, layer);

/*
    //
	// - Fill any rooms of less than 5 spaces
	//
	for (const auto& keyNvalues : floorMaps.second) {
		if (keyNvalues.second.size() < 5) {
			for (const auto& floor : keyNvalues.second) {
				Vector2i corner = getMapPos(floor.x,floor.y);
				setCell(pTileMap, layer, corner, info.mWall);
			}
		}
	}
*/

	//
	// Find all the rooms
	//
	std::pair< Vector2iIntMap, IntVectorOfVector2iMap > floorMaps = findRooms(pTileMap, layer);

	//
	// Remove thinnest walls between room
	//
	joinRooms(pTileMap, layer, floorMaps);

	info.pTileMap = pTileMap;
	info.mLayer = layer;
	Cave::CaveSmoother smoother(info);
	smoother.smoothEdges();

	//
	// Remove cells just touching on diagonal etc
	//
//	fixup(pTileMap, layer);
}

// ===========================================================================

bool GDCave::isWall(int cx, int cy, TileMap* pTileMap, int layer) {
	if (cx >= 0 && cx < info.mCaveWidth && cy >= 0 && cy < info.mCaveHeight) {
		Vector2i coords = getMapPos(cx,cy);
		return (pTileMap->get_cell_atlas_coords(layer,coords, false) == info.mWall);
	}
	return false;
}

bool GDCave::isFloor(int cx, int cy, TileMap* pTileMap, int layer) {
	if (cx >= 0 && cx < info.mCaveWidth && cy >= 0 && cy < info.mCaveHeight) {
		Vector2i coords = getMapPos(cx,cy);
		return (pTileMap->get_cell_atlas_coords(layer,coords, false) == info.mFloor);
	}
	return false;
}

// ===========================================================================

struct GDCave::BorderWall {
    Vector2i floor1; // Room1 floor
    Vector2i floor2; // Room2 floor
    Vector2i dir;    // Wall direction
    int room1;         // First room ID
    int room2;         // Second room ID
    int thickness;     // Distance through the wall
};

std::vector<GDCave::BorderWall> GDCave::detectBorderWalls(TileMap* pTileMap, int layer,
		std::pair<Vector2iIntMap, IntVectorOfVector2iMap> floorMaps)
{
	std::vector<BorderWall> borderWalls;
	Vector2iIntMap floorToRoomMap = floorMaps.first;
	IntVectorOfVector2iMap roomsMap = floorMaps.second;

	std::vector<int> checkedRooms;

	for (const auto& [roomID, tiles] : roomsMap) {
		checkedRooms.push_back(roomID);
		for (const auto& tile : tiles) {
			// Check the 4 neighbors (left, right, up, down)
			for (const auto& [dx, dy] : {std::pair{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
				int cx = tile.x + dx;
				int cy = tile.y + dy;
				if (isWall(cx, cy, pTileMap, layer)) {
					std::set<int> adjacentRooms;
					adjacentRooms.insert(roomID); // The current tile's room

					// Traverse the wall thickness until find a floor (or boundary)
					int thickness = 0;
					while (isWall(cx,cy, pTileMap, layer)) {
						cx += dx;
						cy += dy;
						thickness++;
					}

					// Use the map of Floor => RoomID to check if new room
					if (isFloor(cx,cy,pTileMap,layer)) {
						auto it = floorToRoomMap.find( Vector2i(cx,cy) );
						assert(it != floorToRoomMap.end()); // "ERROR: Floor but not in floorToRoomMap"

						// Check the room for this floor hasn't been checked already
						// NOTE: Current room has already been added, so if no found then
						// also know it is a different room
						int otherRoomID = it->second;
						if (std::find(checkedRooms.begin(), checkedRooms.end(), otherRoomID) == checkedRooms.end()) {
							adjacentRooms.insert(otherRoomID);
						}
					}

					if (adjacentRooms.size() == 2) { // Found a border wall
						auto it = adjacentRooms.begin();
						int room1 = *it++;
						int room2 = *it;
						// The wall just between entering other room
						Vector2i wallPos(tile.x + dx * thickness, tile.y + dy *thickness);
						borderWalls.push_back({ tile, Vector2i(cx,cy), Vector2i(dx,dy), room1, room2, thickness});
					}
				}
			}
		}
	}

	return borderWalls;
}

std::vector<GDCave::BorderWall> GDCave::findMST_Kruskal(std::vector<GDCave::BorderWall>& borderWalls, std::vector<int> roomIds) {
    std::vector<BorderWall> mst; // To store edges in the MST
    Algo::DisjointSets<int> dsu;
    const int numRooms = roomIds.size();

    // Step 1: Add all nodes to the DSU
    for (int i : roomIds) {
        dsu.addElement(i);
    }

    // Step 1: Sort edges by cost
    std::sort(borderWalls.begin(), borderWalls.end(), [](const BorderWall& a, const BorderWall& b) {
        // Sort by cost
        return a.thickness  < b.thickness;
    });
	std::cerr << "=== START MST: " << borderWalls.size() << " " << numRooms << std::endl;

    // Step 2: Process edges in sorted order
    for (const BorderWall& wall : borderWalls) {
    	// Find the sets of the two vertices
    	int setU = dsu.findSet(wall.room1);
    	int setV = dsu.findSet(wall.room2);

        // Add the edge if it connects two different sets
        if (setU != setV) {
            mst.push_back(wall);
            dsu.joinSets(setU, setV);
            std::cerr << " JOIN " << setU << " " << setV << std::endl;
        }
        // Stop if MST contains enough edges
        if (mst.size() == numRooms - 1) break;
    }
	std::cerr << "DONE MST: " << mst.size() << std::endl;
	for (auto& node : mst) {
		std::cerr <<" BORDER: r1=" << node.room1 << " r2=" << node.room2 << " thick="
				<< node.thickness << " wall=" << node.dir.x << "," << node.dir.y
				<< std::endl;
	}

    return mst;
}

// ===========================================================================
//
// Find the adcagent rooms and remove the thinest
//
void GDCave::joinRooms(TileMap* pTileMap, int layer, std::pair<Vector2iIntMap, IntVectorOfVector2iMap> floorMaps)
{
	std::vector<GDCave::BorderWall> borderWalls = detectBorderWalls(pTileMap, layer, floorMaps);
	IntVectorOfVector2iMap roomToFloorsMap = floorMaps.second;

	std::vector<int> roomIds;
	for(auto p : roomToFloorsMap) {
		roomIds.push_back(p.first);
	}
	std::vector<GDCave::BorderWall> mst = findMST_Kruskal(borderWalls, roomIds);
	for (auto& node : mst) {
		int wx = node.floor1.x + node.dir.x;
		int wy = node.floor1.y + node.dir.y;
		for (int i=0; i < node.thickness; ++i) {
			if (isFloor(wx, wy, pTileMap, layer)) {
				std::cerr <<  "ERROR: Wall to join room is ALREADY FLOOR: " << wx << "," << wy << std::endl;
			}
			Vector2i coords = getMapPos(wx, wy);
			setCell(pTileMap, layer, coords, info.mFloor);
			wx += node.dir.x;
			wy += node.dir.y;
		}

	}
}

// ===========================================================================
//
// Remove a map of floor -> room ID
// and room ID -> floor tiles
//
std::pair< GDCave::Vector2iIntMap, GDCave::IntVectorOfVector2iMap >
GDCave::findRooms(TileMap* pTileMap, int layer)
{
    Algo::DisjointSets<Vector2i> floors;
    // Directions for 4-connected neighbors: right, down, left, up.
    static const std::vector<Vector2i> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    // Initialize floors
    for (int cx = 0; cx < info.mCaveWidth; ++cx) {
        for (int cy = 0; cy < info.mCaveHeight; ++cy) {
			Vector2i coords = getMapPos(cx,cy);
			Vector2i t = pTileMap->get_cell_atlas_coords(layer,coords,false);
            if (t == info.mFloor) {
            	floors.addElement(Vector2i(cx,cy));
            }
        }
    }

    // Union adjacent zero cells.
    for (int cx = 0; cx < info.mCaveWidth; ++cx) {
    	for (int cy = 0; cy < info.mCaveHeight; ++cy) {
    		Vector2i coords = getMapPos(cx,cy);
    		Vector2i t = pTileMap->get_cell_atlas_coords(layer,coords,false);
    		if (t == info.mFloor) {
    			for (const Vector2i& dir : directions) {
    				int nx = cx + dir.x;
    				int ny = cy + dir.y;
    				if (   (nx >= 0 && nx < info.mCaveWidth)
    					&& (ny >= 0 && ny < info.mCaveHeight) ) {
    					Vector2i neighbor = getMapPos(nx,ny);
    					Vector2i dirTile = pTileMap->get_cell_atlas_coords(layer,neighbor,false);
    					if (dirTile == info.mFloor) {
    						int i1 = floors.findSet(Vector2i(cx,cy));
    						int i2 = floors.findSet(Vector2i(nx,ny));
    						floors.joinSets(i1, i2);

    					}
    				}
    			}
    		}
    	}
    }

    // Create mappings: grid_to_set and set_to_cells.
    Vector2iIntMap grid_to_set;          // Maps (x, y) to its set ID.
    IntVectorOfVector2iMap set_to_cells;  // Maps set ID to list of cells.

    for (int cx = 0; cx < info.mCaveWidth; ++cx) {
        for (int cy = 0; cy < info.mCaveHeight; ++cy) {
        	Vector2i coords = getMapPos(cx,cy);
        	Vector2i t = pTileMap->get_cell_atlas_coords(layer,coords,false);
            if (t == info.mFloor) {
                Vector2i current(cx, cy);
                int rootId = floors.findSet( current );
                grid_to_set[current] = rootId;
                set_to_cells[rootId].push_back(current);
            }
        }
    }

    return std::pair(grid_to_set, set_to_cells);
}


///////////////////////////////////////////////////

//
// Fix up
// Count the DIAGs and NSEW and decide if the tile
// should be changed
//
void GDCave::fixup(TileMap* pTileMap, int layer)
{
	std::vector<Vector2i> walls;
	std::vector<Vector2i> floors;
	// Pick 10 as max loop count. Returns which no change
	for (int lp=0; lp < 10; ++lp) {
		for (int cy=0; cy < info.mCaveHeight; ++cy) {
			for (int cx=0; cx < info.mCaveWidth; ++cx) {
				Vector2i coords = getMapPos(cx,cy);
				int count = 0;
				int DIAG = 0;
				int NSEW = 0;
				// NW/N/NE
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(-info.mCellWidth,-info.mCellHeight),false) == info.mWall) {
					++count;
					DIAG += 1;
				}
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(0,-info.mCellHeight),false) == info.mWall) {
					++count;
					NSEW += 1;
				}
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(info.mCellWidth,-info.mCellHeight),false) == info.mWall) {
					++count;
					DIAG += 2;
				}

				// E/W
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(info.mCellWidth,0),false) == info.mWall) {
					++count;
					NSEW += 2;
				}
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(-info.mCellWidth,0),false) == info.mWall) {
					++count;
					NSEW += 8;
				}

				// SW/S/SE
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(-info.mCellWidth,info.mCellHeight),false) == info.mWall) {
					++count;
					DIAG += 8;
				}
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(0,info.mCellHeight),false) == info.mWall) {
					NSEW += 4;
				}
				if (pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(info.mCellWidth,info.mCellHeight),false) == info.mWall) {
					DIAG == 4;
				}

				if (pTileMap->get_cell_atlas_coords(layer,coords,false) == info.mWall) {
					// NW but not N+W == blank
					if (((DIAG&0b0001) != 0) && ((NSEW&0b1001) == 0)) {
						floors.push_back(coords);
					}
					// NE but not N+E == blank
					if (((DIAG&0b0010) != 0) && ((NSEW&0b0011) == 0)) {
						floors.push_back(coords);
					}
					// SE but not S+E == blank
					if (((DIAG&0b0100) != 0) && ((NSEW&0b0110) == 0)) {
						floors.push_back(coords);
					}
					// SW but not S+W == blank
					if (((DIAG&0b1000) != 0) && ((NSEW&0b1100) == 0)) {
						floors.push_back(coords);
					}
				}
				if (pTileMap->get_cell_atlas_coords(layer,coords,false) == info.mFloor) {
					// NW but not N+W == blank
					if ((DIAG = 0b1111) && (NSEW == 0b1111)) {
						walls.push_back(coords);
					}
				}
			}
		}
		if (walls.empty() && floors.empty()) {
			return;
		}
		for (Vector2i corner : walls) {
			setCell(pTileMap, layer, corner, info.mWall);
		}
		for (Vector2i corner : floors) {
			setCell(pTileMap, layer, corner, info.mFloor);
		}
		walls.clear();
		floors.clear();
	}
}

///////////////////////////////////////////////////

void GDCave::setCell(TileMap* pTileMap, int layer, Vector2i coords, Vector2i tile) {
	Vector2i corner = Vector2i(coords.x, coords.y);
	for (int y=0; y < info.mCellHeight; ++y) {
		for (int x=0; x < info.mCellWidth; ++x) {
			Vector2i pos(corner.x + x, corner.y + y);
			pTileMap->set_cell(layer,pos,0,tile,0);
		}
	}
}

///////////////////////////////////////////////////
