#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>
#include <GDCave.hpp>
#include "../../cave/inc/RandSimple.h"

using namespace godot;

void GDCave::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_cave_size", "caveSize"), &GDCave::setCaveSize);
	ClassDB::bind_method(D_METHOD("set_border_cell_size", "cells"), &GDCave::setBorderCellSize);
	ClassDB::bind_method(D_METHOD("set_start_cell", "x", "y"), &GDCave::setStartCell);
	ClassDB::bind_method(D_METHOD("set_floor", "floorTileCoords"), &GDCave::setFloor);
	ClassDB::bind_method(D_METHOD("set_wall", "wallTileCoords"), &GDCave::setWall);
	ClassDB::bind_method(D_METHOD("make_cave", "pTileMap", "layer", "seed"), &GDCave::make_it);
}

GDCave::GDCave() {
	mFloor = Vector2i(0,0);
	mWall = Vector2i(0,1);
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

void GDCave::make_it(TileMap* pTileMap, int layer, int seed) {

	RNG::RandSimple simple(seed);
	// Make the border
	// - Top/Bottom
	for (int cy=0; cy < mBorderHeight; ++cy) {
		for (int cx=0; cx < mBorderWidth*2+mCaveWidth; ++cx) {
			Vector2i coordsTop(cx, cy);
			Vector2i coordsBtm(cx, cy+mCaveHeight+mBorderHeight);
			pTileMap->set_cell(layer,coordsTop,0,mWall,0);
			pTileMap->set_cell(layer,coordsBtm,0,mWall,0);
		}
	}
	// - Left/Right
	for (int cx=0; cx < mBorderWidth; ++cx) {
		for (int cy=0; cy < mBorderHeight*2+mCaveHeight; ++cy) {
			Vector2i coordsLft(cx, cy);
			Vector2i coordsRgt(cx+mCaveWidth+mBorderWidth, cy);
			pTileMap->set_cell(layer,coordsLft,0,mWall,0);
			pTileMap->set_cell(layer,coordsRgt,0,mWall,0);
		}
	}
	for (int cy=0; cy < mBorderHeight; ++cy) {
		for (int cx=0; cx < mBorderWidth; ++cx) {
			Vector2i coordsTop(cx, cy);
			Vector2i coordsBtm(cx, cy+mCaveHeight);
			pTileMap->set_cell(layer,coordsTop,0,mWall,0);
			pTileMap->set_cell(layer,coordsBtm,0,mWall,0);
			coordsTop.x += mCaveWidth;
			coordsBtm.x += mCaveWidth;
			pTileMap->set_cell(layer,coordsTop,0,mWall,0);
			pTileMap->set_cell(layer,coordsBtm,0,mWall,0);
		}
	}
	// Fill with random
	for (int cy=mBorderHeight; cy < mBorderHeight+mCaveHeight; ++cy) {
		for (int cx=mBorderWidth; cx < mBorderWidth+mCaveWidth; ++cx) {
			Vector2i tile = (simple.getInt(0,99) < 45) ? mWall : mFloor;
			Vector2i coords(cx, cy);
			pTileMap->set_cell(layer,coords,0,tile,0);
		}
	}
}

///////////////////////////////////////////////////

///////////////////////////////////////////////////
