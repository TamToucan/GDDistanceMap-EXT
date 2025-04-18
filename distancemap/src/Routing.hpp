#ifndef _ROUTING_HPP
#define _ROUTING_HPP

#include <vector>
#include "GridTypes.hpp"

namespace Routing {

struct SparseGraph {
    std::vector<std::vector<std::pair<int, int>>> forward_adj;  // node -> {neighbor, edge_idx}
    std::vector<std::vector<std::pair<int, int>>> reverse_adj;
    std::vector<int> edge_costs;
    std::vector<GridType::Point> edge_endpoints;
    std::vector<GridType::Point> node_coords;
};

SparseGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes,
 const std::vector<GridType::Edge>& baseEdges);

std::vector<int> findZonePath(const SparseGraph routingGraph,
	const std::vector<int>& zoneBases,
	const std::vector<int>& zoneEdges,
	int sourceEdgeIdx,
	int targetEdgeIdx);

}

#endif