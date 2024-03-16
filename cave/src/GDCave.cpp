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

using namespace godot;

void GDCave::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cave_size", "caveSize"), &GDCave::setCaveSize);
	ClassDB::bind_method(D_METHOD("set_border_cell_size", "cells"), &GDCave::setBorderCellSize);
	ClassDB::bind_method(D_METHOD("set_cell_size", "cells"), &GDCave::setCellSize);
	ClassDB::bind_method(D_METHOD("set_start_cell", "x", "y"), &GDCave::setStartCell);
	ClassDB::bind_method(D_METHOD("set_floor", "floorTileCoords"), &GDCave::setFloor);
	ClassDB::bind_method(D_METHOD("set_wall", "wallTileCoords"), &GDCave::setWall);
	ClassDB::bind_method(D_METHOD("set_octaves", "octaves"), &GDCave::setOctaves);
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

void GDCave::make_it(TileMap* pTileMap, int layer, int seed) {

	RNG::RandSimple simple(seed);
	// Make the border
	// - Top/Bottom
	for (int cy=0; cy < mBorderHeight; ++cy) {
		for (int cx=0; cx < mBorderWidth*2+(mCaveWidth*mCellWidth); ++cx) {
			Vector2i coordsTop(cx, cy);
			Vector2i coordsBtm(cx, cy+mCaveHeight*mCellHeight+mBorderHeight);
			pTileMap->set_cell(layer,coordsTop,0,mWall,0);
			pTileMap->set_cell(layer,coordsBtm,0,mWall,0);
		}
	}
	// - Left/Right
	for (int cx=0; cx < mBorderWidth; ++cx) {
		for (int cy=0; cy < mBorderHeight*2+mCaveHeight*mCellHeight; ++cy) {
			Vector2i coordsLft(cx, cy);
			Vector2i coordsRgt(cx+mCaveWidth*mCellWidth+mBorderWidth, cy);
			pTileMap->set_cell(layer,coordsLft,0,mWall,0);
			pTileMap->set_cell(layer,coordsRgt,0,mWall,0);
		}
	}

	// Fill with random
	double W = mCaveWidth;
	double H = mCaveHeight;
	//std::stringstream ss;
    // ss << "CAVE with octaves: " << mOctaves;
    // ERR_PRINT(ss.str().c_str());

	for (int cy=0; cy < mCaveHeight; ++cy) {
		for (int cx=0; cx < mCaveWidth; ++cx) {
			Vector2i coords(mBorderWidth+cx*mCellWidth, mBorderHeight+cy*mCellHeight);
			Vector2i tile = (Algo::getNoise2(cx/W, cy/H, mOctaves) < 0.0) ? mWall : mFloor;
			//Vector2i tile = (simple.getInt(0,99) < 45) ? mWall : mFloor;
			setCell(pTileMap, layer,coords,tile);
		}
	}
}

///////////////////////////////////////////////////
void GDCave::setCell(TileMap* pTileMap, int layer, Vector2i coords, Vector2i tile) {
	for (int y=0; y < mCellHeight; ++y) {
		for (int x=0; x < mCellWidth; ++x) {
			Vector2i pos(coords.x + x, coords.y + y);
			pTileMap->set_cell(layer,pos,0,tile,0);
		}
	}
}

///////////////////////////////////////////////////
