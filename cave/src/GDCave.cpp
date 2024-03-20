#include <sstream>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>
#include <GDCave.hpp>
#include "RandSimple.h"
#include "DJSets.h"
#include "PerlinNoise.h"
#include "SimplexNoise.h"
#include "noise1234.h"

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
	ClassDB::bind_method(D_METHOD("set_freq", "freq"), &GDCave::setFreq);
	ClassDB::bind_method(D_METHOD("set_amp", "amp"), &GDCave::setAmp);
	ClassDB::bind_method(D_METHOD("set_loops", "loops"), &GDCave::setLoops);
	ClassDB::bind_method(D_METHOD("make_cave", "pTileMap", "layer", "seed"), &GDCave::make_it);
}

GDCave::GDCave() {
	mFloor = Vector2i(0,0);
	mWall = Vector2i(0,1);

   Algo::DJSets sets;
   sets.AddElements(10);
}

GDCave::~GDCave() {
}


GDCave* GDCave::setCaveSize(Vector2i caveSize) {
	mCaveWidth = caveSize.x;
	mCaveHeight = caveSize.y;
	return this;
}

GDCave* GDCave::setBorderCellSize(Vector2i cells) {
	mBorderWidth = cells.x;
	mBorderHeight = cells.y;
	return this;
}

GDCave* GDCave::setCellSize(Vector2i cells) {
	mCellWidth = cells.x;
	mCellHeight = cells.y;
	return this;
}

GDCave* GDCave::setStartCell(int x, int y) {
	mStartCellX = x;
	mStartCellY = y;
	return this;
}

GDCave* GDCave::setFloor(godot::Vector2i floor) {
	mFloor = floor;
	return this;
}

GDCave* GDCave::setWall(godot::Vector2i wall) {
	mWall = wall;
	return this;
}

GDCave* GDCave::setOctaves(int octaves) {
	mOctaves = octaves;
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

GDCave* GDCave::setLoops(int loops) {
	mLoops = loops;
	return this;
}

void GDCave::make_it(TileMap* pTileMap, int layer, int seed) {

	RNG::RandSimple simple(seed);
	// So don't need to worry about -1,-1 adjust from 0,0 pos when counting neighbors
	if (mCellWidth > mBorderWidth || mCellHeight > mBorderHeight) {
		std::stringstream ss;
		ss << "Cell Width&Height must be <= Border Width&Height";
		ERR_PRINT(ss.str().c_str());
		return;
	}
	// Make the border
	// - Top/Bottom
	for (int cy=0; cy < mBorderHeight; ++cy) {
		for (int cx=0; cx < 2*mBorderWidth +(mCaveWidth*mCellWidth); ++cx) {
			Vector2i coordsTop(cx, cy);
			Vector2i coordsBtm(cx, cy+mCaveHeight*mCellHeight+mBorderHeight);
			pTileMap->set_cell(layer,coordsTop,0,mWall,0);
			pTileMap->set_cell(layer,coordsBtm,0,mWall,0);
		}
	}
	// - Left/Right
	for (int cx=0; cx < mBorderWidth; ++cx) {
		for (int cy=0; cy < 2*mBorderHeight +(mCaveHeight*mCellHeight); ++cy) {
			Vector2i coordsLft(cx, cy);
			Vector2i coordsRgt(cx+mCaveWidth*mCellWidth+mBorderWidth, cy);
			pTileMap->set_cell(layer,coordsLft,0,mWall,0);
			pTileMap->set_cell(layer,coordsRgt,0,mWall,0);
		}
	}

	// Fill with random
	double W = mCaveWidth-1 +mAmp;
	double H = mCaveHeight-1 +mAmp;
	int wallCount = 0;
	double (*pf)(double, double, int) = mPerlin ? &Algo::getNoise2 : &Algo::getSNoise2;
	for (int cy=0; cy < mCaveHeight; ++cy) {
		for (int cx=0; cx < mCaveWidth; ++cx) {
			Vector2i coords(mBorderWidth+cx*mCellWidth, mBorderHeight+cy*mCellHeight);
			double x = cx/W *mFreq;
			double y = cy/H *mFreq;
			double n1 = (*pf)(x,y,mOctaves);
			if (simple.getFloat() > 0.55) {
			//XXXif (n1 < 0) {
				++wallCount;
				setCell(pTileMap, layer,coords,mWall);
			}
			else {
				setCell(pTileMap, layer,coords,mFloor);
			}
		}
	}

	// Cellular automata
	std::vector<Vector2i> walls(mCaveHeight*mCaveWidth/2);
	std::vector<Vector2i> floors(mCaveHeight*mCaveWidth/2);
	for (int lp=0; lp < mLoops; ++lp) {
		std::stringstream ss;
		ss << "LP: " << lp << " WALLS: " << wallCount;
		ERR_PRINT(ss.str().c_str());
		for (int cy=0; cy < mCaveHeight; ++cy) {
			for (int cx=0; cx < mCaveWidth; ++cx) {
				Vector2i coords(mBorderWidth+cx*mCellWidth, mBorderHeight+cy*mCellHeight);
				// If this is a floor the count the walls around it
				int count = pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(-mCellWidth,-mCaveHeight),false) == mWall ? 1 : 0;
				count += pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(0,-mCaveHeight),false) == mWall ? 1 : 0;
				count += pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(mCaveWidth,-mCaveHeight),false) == mWall ? 1 : 0;

				count += pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(-mCaveWidth,0),false) == mWall ? 1 : 0;
				count += pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(mCaveWidth,0),false) == mWall ? 1 : 0;

				count += pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(-mCaveWidth,mCaveHeight),false) == mWall ? 1 : 0;
				count += pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(0,mCaveHeight),false) == mWall ? 1 : 0;
				count += pTileMap->get_cell_atlas_coords(layer,coords+Vector2i(mCaveWidth,mCaveHeight),false) == mWall ? 1 : 0;
				if (pTileMap->get_cell_atlas_coords(layer,coords,false) == mFloor) {
					if (count >= 5 || count == 0) {
						//setCell(pTileMap, layer, coords, mWall);
						walls.push_back(coords);
						++wallCount;
					}
				}
				else {
					if (count < 5 && count != 0) {
						//setCell(pTileMap, layer, coords, mFloor);
						floors.push_back(coords);
						--wallCount;
					}
				}
			}
		}
		for (Vector2i corner : walls) {
			setCell(pTileMap, layer, corner, mWall);
		}
		for (Vector2i corner : walls) {
			setCell(pTileMap, layer, corner, mFloor);
		}
		walls.clear();
		floors.clear();
	}
	std::stringstream ss;
	ss << "WALLS: " << wallCount;
	ERR_PRINT(ss.str().c_str());
}

///////////////////////////////////////////////////

void GDCave::setCell(TileMap* pTileMap, int layer, Vector2i coords, Vector2i tile) {
	Vector2i corner = Vector2i(coords.x, coords.y);
	for (int y=0; y < mCellHeight; ++y) {
		for (int x=0; x < mCellWidth; ++x) {
			Vector2i pos(corner.x + x, corner.y + y);
			pTileMap->set_cell(layer,pos,0,tile,0);
		}
	}
}

///////////////////////////////////////////////////
