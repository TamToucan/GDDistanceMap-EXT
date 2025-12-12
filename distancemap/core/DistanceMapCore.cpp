#include "DistanceMapCore.hpp"
#include "Debug.h"
#include "GridToGraph.hpp"

namespace DistanceMap {

void DistanceMapCore::initialize(const std::vector<std::vector<int>> &grid,
                                 const Router::Info &info) {
  SET_DEBUG("ALL");
  this->info = info;

  LOG_DEBUG("## CREATE DistaceMapCore");
  for (const auto &row : grid) {
    for (int xy : row) {
      LOG_DEBUG_CONT((xy ? '#' : ' '));
    }
    LOG_DEBUG("");
  }
  LOG_DEBUG("");

  // Make a grid of distance to closest wall (0 = wall)
  LOG_INFO("## makeWallDistanceGrid");
  wallDistGrid = DistanceMap::makeWallDistanceGrid(grid);

  // Make a SightGrid which can return the distance to wall (N,S,E,W)
  LOG_INFO("## makeSightGrid");
  sightGrid = DistanceMap::makeSightGrid(grid);

  // Create a floor grid where
  //   WALLS = EMPTY
  //   FLOOR = PATH
  GridType::Grid floorGrid = GridToGraph::gridToFloorGrid(grid);

  // Use that floorGrid to create the complete graph for movement
  LOG_INFO("## makeGraph");
  auto graph = GridToGraph::makeGraph(floorGrid);
  navGraph.initialize(graph, info);
  distMapNav.initialize(graph.infoGrid, info);
}

float DistanceMapCore::getMove(Router::RouteCtx *ctx, GridType::Vec2 from,
                               GridType::Vec2 to, int type) {
  return navGraph.getMoveDirection(ctx, from, to, type);
}
} // namespace DistanceMap
