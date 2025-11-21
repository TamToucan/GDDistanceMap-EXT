#ifndef NAVIGATION_GRAPH_HPP
#define NAVIGATION_GRAPH_HPP

#include <vector>
#include <tuple>
#include <godot_cpp/variant/vector2.hpp>
#include "GridTypes.hpp"
#include "SparseNavGraph.hpp"
#include "Router.hpp"
#include "GridToGraph.hpp"
#include "GDDistanceMapApi.h"

namespace Routing {

class GDDISTANCE_MAP_API NavigationGraph {
public:
    NavigationGraph();
    ~NavigationGraph() = default;

    void initialize(const GridToGraph::Graph& graphData, const Router::Info& info);
    
    // Main entry point for movement
    float getMoveDirection(Router::RouteCtx* ctx, godot::Vector2 from, godot::Vector2 to, int type);

private:
    // Data members from GridToGraph::Graph
    GridType::Grid m_infoGrid;
    GridToGraph::BaseGraph m_baseGraph;
    GridToGraph::PathCostMap m_pathCostMap;
    SparseNavGraph m_routingGraph;
    std::vector<GridType::Edge> m_baseEdges;
    std::vector<GridType::Point> m_baseNodes;
    std::vector<GridType::Point> m_deadEnds;
    std::vector<GridToGraph::AbstractLevel> m_abstractLevels;

    Router::Info m_info;

    // Helpers
    GridType::Point getNextMove(Router::RouteCtx* ctx, GridType::Point source, GridType::Point target);
    std::tuple<int, int, bool> getClosestNode(const GridType::Point& pos);
    GridType::Point nextStep(const GridType::Point& from, const GridType::Point& to);
    GridType::Point nextPoint(const GridType::Point& from, const GridType::Point& dir);
    float computeAngle(double dx, double dy);
};

}

#endif
