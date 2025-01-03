#ifndef DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_
#define DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_

#include <vector>

namespace DistanceMap {

//
// Function to compute the distance grid
// Distance grid is a new grid the size of the input which has
// distance to the nearest wall (0 if it is a wall)
//
std::vector<std::vector<int>> makeWallDistanceGrid(const std::vector<std::vector<int>>& grid);

}



#endif /* DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_ */
