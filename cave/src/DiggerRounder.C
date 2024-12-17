#ifdef DONT_HIDE

#include <iterator>
#include <algorithm>
#include <functional>

#include "DiggerRounder.h"

#define ROUND_LOG

namespace Cave {

namespace {

//////////////////////////////////////
// Based on Cherry::TileRounder
//////////////////////////////////////

//
// Here's what I think I wrote. The TileGrid's are a 4x4 of tiles where
//   X = don't care
//   B = blank
//   S = set
//   N = loc of 1st tile to change
//   M = loc of 2nd tile to change (if this is a two tile update)
//
// The last letter of the TileGrid name means
//   n = normal
//   u = shifted up one row (top was X,X,X,X)
//   d = shifted down one row (bottom was X,X,X,X)
//   l = shifted left one col (left col was X,X,X,X)
//   r = shifted right one col (right col was X,X,X,X)
//
// The TileUpdate holds a pointer to the TileGrid and the two tiles to replace
// the 'N' and 'M' with. The s_tileUpdates[] is then a list of all the updates
// to check for a match with in the order to check them (2 tile updates need
// checked first because the single tile update is more general and will also
// match what should be a 2 tile update)
//
// The createUpdateInfos parses the s_tileUpdates to generate a list of RoundTileInfo
// records i.e. the above is all just to specify the updates in a "friendly" way. The
// RoundTileInfo is the machine-friendly format. It has a mask and value based on the
// 'S','B','X' values as well as the X,Y offset to the 1st and 2nd tiles to update.
//
// The roundEdges iterates over each edge cell and calc's the value for the 4x4 grid
// around it. The list of RoundTileInfo is then searched to find a match and the update
// generated. Updates are put into the TileMapUpdateManager and for an update to go
// ahead neither tiles (N and M) can have be previously updated.
//
// The TileMapUpdateManager takes the place of the MapSlice in Cherry:TileRounder
// 
// The list of RounderTileInfo records must be checked bottom to top, left to right,
// so the list of edges is first sorted.
//
const int GRD_W = 4;
const int GRD_H = 4;
const unsigned char X = 'x';
const unsigned char S = 's';
const unsigned char B = 'b';
const unsigned char N = 'n';
const unsigned char M = 'm';
// Two tile updates
unsigned char TileGrid00n[GRD_H][GRD_W] = { {X,S,S,S},{S,N,M,B},{X,B,B,X},{X,X,X,X} };
unsigned char TileGrid00d[GRD_H][GRD_W] = { {X,X,X,X},{X,S,S,S},{S,N,M,B},{X,B,B,X} };
unsigned char TileGrid01n[GRD_H][GRD_W] = { {X,X,S,X},{X,B,N,S},{X,B,M,S},{X,X,B,S} };
unsigned char TileGrid01l[GRD_H][GRD_W] = { {X,S,X,X},{B,N,S,X},{B,M,S,X},{X,B,S,X} };
unsigned char TileGrid02n[GRD_H][GRD_W] = { {X,X,X,X},{X,B,B,X},{B,M,N,S},{S,S,S,X} };
unsigned char TileGrid02u[GRD_H][GRD_W] = { {X,B,B,X},{B,M,N,S},{S,S,S,X},{X,X,X,X} };
unsigned char TileGrid03n[GRD_H][GRD_W] = { {S,B,X,X},{S,M,B,X},{S,N,B,X},{X,S,X,X} };
unsigned char TileGrid03r[GRD_H][GRD_W] = { {X,S,B,X},{X,S,M,B},{X,S,N,B},{X,X,S,X} };
unsigned char TileGrid04n[GRD_H][GRD_W] = { {S,S,S,X},{B,M,N,S},{X,B,B,X},{X,X,X,X} };
unsigned char TileGrid04d[GRD_H][GRD_W] = { {X,X,X,X},{S,S,S,X},{B,M,N,S},{X,B,B,X} };
unsigned char TileGrid05n[GRD_H][GRD_W] = { {X,X,B,S},{X,B,M,S},{X,B,N,S},{X,X,S,X} };
unsigned char TileGrid05l[GRD_H][GRD_W] = { {X,B,S,X},{B,M,S,X},{B,N,S,X},{X,S,X,X} };
unsigned char TileGrid06n[GRD_H][GRD_W] = { {X,X,X,X},{X,B,B,X},{S,N,M,B},{X,S,S,S} };
unsigned char TileGrid06u[GRD_H][GRD_W] = { {X,B,B,X},{S,N,M,B},{X,S,S,S},{X,X,X,X} };
unsigned char TileGrid07n[GRD_H][GRD_W] = { {X,S,X,X},{S,N,B,X},{S,M,B,X},{S,B,X,X} };
unsigned char TileGrid07r[GRD_H][GRD_W] = { {X,X,S,X},{X,S,N,B},{X,S,M,B},{X,S,B,X} };
// Single 45 degree tile updates
unsigned char TileGrid08n[GRD_H][GRD_W] = { {X,X,S,X},{X,B,N,S},{X,X,B,X},{X,X,X,X} };
unsigned char TileGrid08d[GRD_H][GRD_W] = { {X,X,X,X},{X,X,S,X},{X,B,N,S},{X,X,B,X} };
unsigned char TileGrid08l[GRD_H][GRD_W] = { {X,X,X,X},{X,S,X,X},{B,N,S,X},{X,B,X,X} };
unsigned char TileGrid09n[GRD_H][GRD_W] = { {X,X,X,X},{X,X,B,X},{X,B,N,S},{X,X,S,X} };
unsigned char TileGrid09l[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{B,N,S,X},{X,S,X,X} };
unsigned char TileGrid10n[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{S,N,B,X},{X,S,X,X} };
unsigned char TileGrid10u[GRD_H][GRD_W] = { {X,B,X,X},{S,N,B,X},{X,S,X,X},{X,X,X,X} };
unsigned char TileGrid11n[GRD_H][GRD_W] = { {X,S,X,X},{S,N,B,X},{X,B,X,X},{X,X,X,X} };
unsigned char TileGrid11r[GRD_H][GRD_W] = { {X,X,S,X},{X,S,N,B},{X,X,B,X},{X,X,X,X} };
unsigned char TileGrid11d[GRD_H][GRD_W] = { {X,X,X,X},{X,S,X,X},{S,N,B,X},{X,B,X,X} };
unsigned char TileGrid12n[GRD_H][GRD_W] = { {X,S,X,X},{B,N,S,X},{X,B,X,X},{X,X,X,X} };
unsigned char TileGrid12d[GRD_H][GRD_W] = { {X,X,X,X},{X,S,X,X},{B,N,S,X},{X,B,X,X} };
unsigned char TileGrid13n[GRD_H][GRD_W] = { {X,X,B,X},{X,B,N,S},{X,X,S,X},{X,X,X,X} };
unsigned char TileGrid13l[GRD_H][GRD_W] = { {X,B,X,X},{B,N,S,X},{X,S,X,X},{X,X,X,X} };
unsigned char TileGrid13d[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{B,N,S,X},{X,S,X,X} };
unsigned char TileGrid14n[GRD_H][GRD_W] = { {X,X,X,X},{X,X,B,X},{X,S,N,B},{X,X,S,X} };
unsigned char TileGrid14u[GRD_H][GRD_W] = { {X,X,B,X},{X,S,N,B},{X,X,S,X},{X,X,X,X} };
unsigned char TileGrid14l[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{S,N,B,X},{X,S,X,X} };
unsigned char TileGrid15n[GRD_H][GRD_W] = { {X,X,X,X},{X,S,X,X},{S,N,B,X},{X,B,X,X} };
unsigned char TileGrid15r[GRD_H][GRD_W] = { {X,X,X,X},{X,X,S,X},{X,S,N,B},{X,X,B,X} };

// End cap tile updates
unsigned char TileGrid16n[GRD_H][GRD_W] = { {X,X,B,S},{X,B,N,S},{X,X,B,S},{X,X,X,X} };
unsigned char TileGrid16dl[GRD_H][GRD_W]= { {X,X,X,X},{X,B,S,X},{B,N,S,X},{X,B,S,X} };
unsigned char TileGrid17n[GRD_H][GRD_W] = { {X,X,X,X},{X,X,B,X},{X,B,N,B},{X,S,S,S} };
unsigned char TileGrid17l[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{B,N,B,X},{S,S,S,X} };
unsigned char TileGrid18n[GRD_H][GRD_W] = { {X,X,X,X},{S,B,X,X},{S,N,B,X},{S,B,X,X} };
unsigned char TileGrid19n[GRD_H][GRD_W] = { {S,S,S,X},{B,N,B,X},{X,B,X,X},{X,X,X,X} };
unsigned char TileGrid19d[GRD_H][GRD_W] = { {X,X,X,X},{S,S,S,X},{B,N,B,X},{X,B,X,X} };
#if 0
unsigned char TileGrid16d[GRD_H][GRD_W] = { {X,X,X,X},{X,X,B,S},{X,B,N,S},{X,X,B,S} };
unsigned char TileGrid17l[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{B,N,B,X},{S,S,S,X} };
unsigned char TileGrid18u[GRD_H][GRD_W] = { {S,B,X,X},{S,N,B,X},{S,B,X,X},{X,X,X,X} };
unsigned char TileGrid19r[GRD_H][GRD_W] = { {X,S,S,S},{X,B,N,B},{X,X,B,X},{X,X,X,X} };
#endif

// Single isolated tile update
unsigned char TileGrid20n[GRD_H][GRD_W] = { {X,B,X,X},{B,N,B,X},{X,B,X,X},{X,X,X,X} };
unsigned char TileGrid20d[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{B,N,B,X},{X,B,X,X} };

// Added for digger (bit sticking off end)
// Not sure why TileRounder doesn't need it
unsigned char TileGrid21n[GRD_H][GRD_W] = { {X,X,X,X},{B,B,X,X},{S,N,B,X},{S,B,X,X} };
unsigned char TileGrid22n[GRD_H][GRD_W] = { {X,X,X,X},{S,S,B,X},{B,N,B,X},{X,B,X,X} };
unsigned char TileGrid23n[GRD_H][GRD_W] = { {X,X,X,X},{X,B,S,X},{B,N,S,X},{X,B,B,X} };
unsigned char TileGrid24n[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{B,N,B,X},{B,S,S,X} };

unsigned char TileGrid25n[GRD_H][GRD_W] = { {X,X,X,X},{S,B,X,X},{S,N,B,X},{B,B,X,X} };
unsigned char TileGrid26n[GRD_H][GRD_W] = { {X,X,X,X},{B,S,S,X},{B,N,B,X},{X,B,X,X} };
unsigned char TileGrid27n[GRD_H][GRD_W] = { {X,X,X,X},{X,B,B,X},{B,N,S,X},{X,B,S,X} };
unsigned char TileGrid28n[GRD_H][GRD_W] = { {X,X,X,X},{X,B,X,X},{B,N,B,X},{S,S,B,X} };

struct TileUpdate {
    const unsigned char (*pTileGrid)[GRD_H][GRD_W];
    Map::TileMap::TileIndex tile1;
    Map::TileMap::TileIndex tile2;
};

// The two tile update i.e. the 22.5 degree line
const Map::TileMap::TileIndex T00A = 18;
const Map::TileMap::TileIndex T00B = 19;
const Map::TileMap::TileIndex T01A = 33;
const Map::TileMap::TileIndex T01B = 25;
const Map::TileMap::TileIndex T02A = 17;
const Map::TileMap::TileIndex T02B = 16;
const Map::TileMap::TileIndex T03A = 24;
const Map::TileMap::TileIndex T03B = 32;
const Map::TileMap::TileIndex T04A = 23;
const Map::TileMap::TileIndex T04B = 22;
const Map::TileMap::TileIndex T05A = 27;
const Map::TileMap::TileIndex T05B = 35;
const Map::TileMap::TileIndex T06A = 20;
const Map::TileMap::TileIndex T06B = 21;
const Map::TileMap::TileIndex T07A = 34;
const Map::TileMap::TileIndex T07B = 26;

// The single tile 45 degree line update
const Map::TileMap::TileIndex T08A = 11;
const Map::TileMap::TileIndex T09A = 8;
const Map::TileMap::TileIndex T10A = 9;
const Map::TileMap::TileIndex T11A = 10;
const Map::TileMap::TileIndex T12A = 11;
const Map::TileMap::TileIndex T13A = 8;
const Map::TileMap::TileIndex T14A = 9;
const Map::TileMap::TileIndex T15A = 10;

/ The 4 end cap tile updates
#if 0
const Map::TileMap::TileIndex T16A = 15;
const Map::TileMap::TileIndex T17A = 12;
const Map::TileMap::TileIndex T18A = 13;
const Map::TileMap::TileIndex T19A = 14;
#else
// Looks better if remove the tile
const Map::TileMap::TileIndex T16A = 0;
const Map::TileMap::TileIndex T17A = 0;
const Map::TileMap::TileIndex T18A = 0;
const Map::TileMap::TileIndex T19A = 0;
#endif
// The single isolated tile
const Map::TileMap::TileIndex T20A = 3;

// Bit sticking off end (remove)
const Map::TileMap::TileIndex T21A = 0;
const Map::TileMap::TileIndex T22A = 0;
const Map::TileMap::TileIndex T23A = 0;
const Map::TileMap::TileIndex T24A = 0;

const Map::TileMap::TileIndex T25A = 0;
const Map::TileMap::TileIndex T26A = 0;
const Map::TileMap::TileIndex T27A = 0;
const Map::TileMap::TileIndex T28A = 0;

const TileUpdate s_tileUpdates[] = {
//    { &TileGrid00n, T00A, T00B},
    { &TileGrid00d, T00A, T00B},

//    { &TileGrid01n, T01A, T01B},
    { &TileGrid01l, T01A, T01B},

    { &TileGrid02n, T02A, T02B},
//    { &TileGrid02u, T02A, T02B},

    { &TileGrid03n, T03A, T03B},
//    { &TileGrid03r, T03A, T03B},

//    { &TileGrid04n, T04A, T04B},
    { &TileGrid04d, T04A, T04B},

//    { &TileGrid05n, T05A, T05B},
    { &TileGrid05l, T05A, T05B},

    { &TileGrid06n, T06A, T06B},
//    { &TileGrid06u, T06A, T06B},

    { &TileGrid07n, T07A, T07B},
//    { &TileGrid07r, T07A, T07B},

    //
    // The single tile updates must be checked after the 2 tile ones
    // since they overlap with some of them. Also, don't check the
    // single tile update which is just shifted left. If a 2 tile
    // doesn't match then the right hand single tile check will catch it
    //

//    { &TileGrid08n, T08A, 0},
//    { &TileGrid08d, T08A, 0},
    { &TileGrid08l, T08A, 0},

//    { &TileGrid09n, T09A, 0},
    { &TileGrid09l, T09A, 0},

    { &TileGrid10n, T10A, 0},
//    { &TileGrid10u, T10A, 0},

//    { &TileGrid11n, T11A, 0},
//    { &TileGrid11r, T11A, 0},
    { &TileGrid11d, T11A, 0},

//    { &TileGrid12n, T12A, 0},
    { &TileGrid12d, T12A, 0},

//    { &TileGrid13n, T13A, 0},
//    { &TileGrid13l, T13A, 0},
    { &TileGrid13d, T13A, 0},

//    { &TileGrid14n, T14A, 0},
//    { &TileGrid14u, T14A, 0},
    { &TileGrid14l, T14A, 0},

    { &TileGrid15n, T15A, 0},
//    { &TileGrid15r, T15A, 0},

    //
    // The end cap updates
    //

//    { &TileGrid16n, T16A, 0},
    { &TileGrid16dl, T16A, 0},
//    { &TileGrid17n, T17A, 0},
    { &TileGrid17l, T17A, 0},
    { &TileGrid18n, T18A, 0},
//    { &TileGrid19n, T19A, 0},
    { &TileGrid19d, T19A, 0},

    // The single isolated tile
//    { &TileGrid20n, T20A, 0}
    { &TileGrid20d, T20A, 0},

    // Bit sitcking off end
    { &TileGrid21n, T21A, 0},
    { &TileGrid22n, T22A, 0},
    { &TileGrid23n, T23A, 0},
    { &TileGrid24n, T24A, 0},

    { &TileGrid25n, T25A, 0},
    { &TileGrid26n, T26A, 0},
    { &TileGrid27n, T27A, 0},
    { &TileGrid28n, T28A, 0}
};

} // anonymouse

//////////////////////////////////////////////////////////////

namespace {

//
// Used to sort the UpdateInfo into bottom left first
// 
struct CompareUpdateFunctor : public std::binary_function<Map::TileMap::UpdateInfo, Map::TileMap::UpdateInfo, bool>
{
    bool operator()( const Map::TileMap::UpdateInfo& lhs, const Map::TileMap::UpdateInfo& rhs)
    {
        // DiggerRounder searches bottom->up, left->right so if LHS is to
        // the left then return true. If same column then return true if below
        //
        if (lhs.x < rhs.x) return true;
        if (lhs.x > rhs.x) return false;
        return (lhs.y < rhs.y);
    }
};

} // Anonymouse

//////////////////////////////////////////////////////////////

DiggerRounder::DiggerRounder()
{
    createUpdateInfos();
}


DiggerRounder::~DiggerRounder()
{
}

void DiggerRounder::createUpdateInfos()
{

    const int NUM_UPDATES = sizeof(s_tileUpdates)/sizeof(TileUpdate);
    for (int i=0; i < NUM_UPDATES; ++i)
    {
        const unsigned char (*l_pTileGrid)[GRD_H][GRD_W] = s_tileUpdates[i].pTileGrid;
        int l_mask = 0;
        int l_value = 0;
        int l_xOff1 = -1;
        int l_yOff1 = -1;
        int l_xOff2 = -1;
        int l_yOff2 = -1;
        int s=(GRD_H*GRD_W)-1;
        for (int r=0; r < GRD_H; ++r)
        {
            for (int c=0; c < GRD_W; ++c)
            {
                switch ((*l_pTileGrid)[r][c])
                {
                case X: l_mask |= 0<<s; l_value |= 0<<s; break;
                case B: l_mask |= 1<<s; l_value |= 0<<s; break;
                case S: l_mask |= 1<<s; l_value |= 1<<s; break;
                case N: l_mask |= 1<<s; l_value |= 1<<s; l_xOff1=c; l_yOff1=(GRD_H-1)-r; break;
                case M: l_mask |= 1<<s; l_value |= 1<<s; l_xOff2=c; l_yOff2=(GRD_H-1)-r; break;
                default: break;
                }
                --s;
            }
        }
        if (l_xOff1 == -1)
        {
            LOG_ABORT("No tile position. Update number " << i);
        }
        RoundTileInfo l_info;
        l_info.m_update1 = UpdateInfo(l_xOff1, l_yOff1, s_tileUpdates[i].tile1);
        l_info.m_update2 = UpdateInfo(l_xOff2, l_yOff2, s_tileUpdates[i].tile2);
        l_info.m_mask = l_mask;
        l_info.m_value = l_value;
        m_updateInfos.push_back(l_info);
#if defined(ROUND_LOG)
        LOG_INFO("ROUND: UPDATE " << std::dec << i << " => M,V " << std::hex << l_mask <<" , " << l_value
                << std::dec << " T=" << l_info.m_update1.tileNum
                << " OFF1=" << l_xOff1 << "," << l_yOff1 << " OFF2=" << l_xOff2 << "," << l_yOff2);
#endif
    }
}

void DiggerRounder::roundEdges(const Cave::Info& i)
{
    // Sort th edges into bottom left -> top right order
    std::sort(edges.begin(), edges.end(), CompareUpdateFunctor());

    //
    // Check the 4x4 grid around each edge cell to see if an update is required
    // 
    for (auto l_itr = edges.begin(); l_itr != edges.end(); ++l_itr)
    {
        // Need to move down and left one in the 4x4 grid (See comment
        // near top)
        int x = (*l_itr).x -1;
        int y = (*l_itr).y -1;
        {
            // TODO: Could use the previous l_value and shift it down?
            // then OR in the new row
            int l_value=0;
            int l_shift=(GRD_H*GRD_W)-1;
            for (int r=GRD_H-1; r >=0; --r)
            {
                for (int c=0; c < GRD_W; ++c)
                {
                    if (not pUpdateManager->isEmpty(x+c,y+r))
                    {
                        l_value |= (1<<l_shift);
                    }
                    --l_shift;
                }
            }
#if defined(ROUND_LOG)
            LOG_INFO("ROUND: X,Y = " << x << "," << y << " => V " << std::hex << l_value << std::dec);
            int D = 0;
#endif

            // TODO: Could check for special case values that won't be in the
            // m_updateInfos e.g. 0 or 0xffff. Or could pre-process the
            // RoundTileInfos and generate a map of all possible values.
            //
            for (auto& l_itr = m_updateInfos.begin(); l_itr != m_updateInfos.end(); ++l_itr)
            {
                const RoundTileInfo& l_info(*l_itr);
                if ( (l_value&l_info.m_mask) == l_info.m_value)
                {
                    UpdateInfo l_update1(l_info.m_update1);
#if defined(ROUND_LOG)
                    LOG_INFO("ROUND: FOUND MATCH " << std::dec << D << " V*M " << std::hex << l_value << "&" << l_info.m_mask
                             << " =>" << (l_value&l_info.m_mask) << std::dec);
#endif
                    l_update1.x += x;
                    l_update1.y += y;
                    UpdateInfo l_update2(l_info.m_update2);
                    l_update2.x += x;
                    l_update2.y += y;
                    // Check that neither tile is due to be updated (rounded)
                    if (not pUpdateManager->hasPendingUpdate(l_update1.x, l_update1.y)
                        && not (l_info.m_update2.tileNum && pUpdateManager->hasPendingUpdate(l_update2.x, l_update2.y)) )
                    {
                        pUpdateManager->addUpdate(l_update1);
#if defined(ROUND_LOG)
                        LOG_INFO("ROUND: FOUND UPDATE1  X,Y = " << l_update1.x << "," << l_update1.y << " Tile " << l_update1.tileNum);
#endif
                        if (l_info.m_update2.tileNum)
                        {
                            pUpdateManager->addUpdate(l_update2);
#if defined(ROUND_LOG)
                            LOG_INFO("ROUND: FOUND UPDATE2  X,Y = " << l_update2.x << "," << l_update2.y << " Tile " << l_update2.tileNum);
#endif
                        }
                    }
#if defined(ROUND_LOG)
                    else
                    {
                        LOG_INFO("ROUND: Already UPDATED  X,Y = " << l_update1.x << "," << l_update1.y << " Tile " << l_update1.tileNum
                                << " AND/OR " << l_update2.x << "," << l_update2.y << " Tile " << l_update2.tileNum);
                    }
#endif
                    y = l_update1.y-1;
                    break;
                }
#if defined(ROUND_LOG)
                ++D;
#endif
            }
        }
    }
}

} // namespace

#endif
