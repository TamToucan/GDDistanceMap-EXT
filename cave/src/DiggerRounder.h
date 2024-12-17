#ifdef NO_HIDE

#ifndef DIGGER_ROUNDER_H
#define DIGGER_ROUNDER_H


#include <vector>
#include <godot_cpp/classes/vector.hpp>

#include "GDCave.hpp"

using namespace godot;

namespace Cave {

class DiggerRounder
{
public:
    DiggerRounder();
    virtual ~DiggerRounder();

    //
    // I_TileMapDiggerRounder interface
    //
    virtual void roundEdges(const Cave::Info& i);

protected:
    virtual void createUpdateInfos();

protected:
    struct UpdateInfo {
    	int x;
    	int y;
    	Vector2 tile;
    	UpdateInfo() : x(-1), y(-1), tile(-1,1) { }
    	UpdateInfo(int nx, int ny, Vector2 t) : x(nx), y(ny), tileNum(t) { }
    };

    struct RoundTileInfo {
        Map::TileMap::UpdateInfo m_update1;
        Map::TileMap::UpdateInfo m_update2;
        int m_mask;
        int m_value;
    };
    typedef std::vector<RoundTileInfo> RoundTileInfos;
    RoundTileInfos m_updateInfos;
};

} // namespace

#endif

#endif
