#ifndef _ROUTING_HPP
#define _ROUTING_HPP

#include <vector>
#include "GridTypes.hpp"

namespace Routing {

struct SparseGraph {
    std::vector<std::vector<std::pair<int, int>>> forward_adj;  // node -> {neighbor, edge_idx}
    std::vector<std::vector<std::pair<int, int>>> reverse_adj;
    std::vector<int> edgeCosts;
    std::vector<std::pair<int,int>> edgeFromTos;
    std::vector<GridType::Point> nodePoints;
};

SparseGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes,
	const std::vector<GridType::Edge>& baseEdges,
    const GridType::Grid& infoGrid);

std::vector<int> findZonePath(const SparseGraph routingGraph,
	const std::vector<int>& zoneBases,
	const std::vector<int>& zoneEdges,
	int sourceEdgeIdx,
	int targetEdgeIdx);

}

#endif