#pragma once
#include <vector>
#include "GridTypes.hpp"
#include "Router.hpp"
#include "NavigationGraph.hpp"
#include "DistanceMapNavigator.hpp"
#include "WallDistanceGrid.hpp"
#include "GDDistanceMapApi.h"

namespace DistanceMap {
class GDDISTANCE_MAP_API DistanceMapCore {
public:
    DistanceMapCore() = default;
    ~DistanceMapCore() = default;

    void initialize(const std::vector<std::vector<int>>& grid, const Router::Info& info);
    float getMove(Router::RouteCtx* ctx, GridType::Vec2 from, GridType::Vec2 to, int type);

private:
    Router::Info info;
    GridType::Grid wallDistGrid;
    DistanceMap::SightGrid sightGrid;
    Routing::NavigationGraph navGraph;
    Routing::DistanceMapNavigator distMapNav;
};
}
