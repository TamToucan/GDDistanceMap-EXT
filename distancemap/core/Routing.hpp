#ifndef _ROUTING_HPP
#define _ROUTING_HPP

#include <vector>
#include <climits>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <optional>
#include <functional>

#include "GridTypes.hpp"
#include "Grid2D.h"
#include "SparseNavGraph.hpp"

#ifdef _WIN32
#ifdef GDDISTANCEMAP_EXPORTS  // Must match your project name
#define GD_API __declspec(dllexport)
#else
#define GD_API __declspec(dllimport)
#endif
#else
#define GD_API
#endif

namespace Router { struct RouteCtx; }

namespace DistanceMap {
namespace Routing {

// Core graph building functions
MathStuff::Grid2D<uint32_t> makeEdgeGrid(const std::vector<GridType::Edge> edges, const GridType::Grid& grid);

} // namespace Routing
}

#endif // ROUTING_HPP
       //
       //
       //
       //
       //
       //
       //
       //
       //
       //
