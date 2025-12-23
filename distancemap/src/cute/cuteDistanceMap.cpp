#include <vector>
#include <cute.h>
#include "DistanceMapCore.hpp"
#include "Router.hpp"

// This class is just a wrapper to prove both libraries can talk to each other
class CuteDistanceMap {

public:
    CuteDistanceMap() {
            std::vector< std::vector<int>> grid = {
                    { '#','#','#','#','#','#','#','#' },
                    { '#',' ',' ',' ',' ',' ',' ','#' },
                    { '#','#','#',' ','#','#','#','#' },
                    { '#','#','#',' ','#','#','#','#' },
                    { '#',' ','#',' ',' ',' ',' ','#' },
                    { '#','#','#',' ','#','#','#','#' },
                    { '#','#','#','#',' ',' ',' ','#' },
                    { '#','#','#','#','#','#','#','#' }
            };
            DistanceMap::Router::Info info;
            info.mCaveHeight = 8;
            info.mCellWidth = 8;
            info.mCellHeight = 8;
            DistanceMap::DistanceMapCore core;
            core.initialize(grid, info);
        }
    ~CuteDistanceMap() {
    }

    void draw() {
	    Cute::draw_push_color(Cute::color_white());
	    Cute::draw_line(Cute::v2(0,0), Cute::v2(100, 100), 2.0f);
    }

private:
    DistanceMap::DistanceMapCore core;
};
