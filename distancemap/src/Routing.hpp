#ifndef _ROUTING_HPP
#define _ROUTING_HPP

#include <vector>
#include <cstdint>
#include "GridTypes.hpp"
#include "Grid2D.h"

namespace Routing {

	// Use to store supset of base nodes/edges in format that can be used for routing
	// - forward/backward give the node to adjacent nodes, along with the edge index
struct SparseGraph {
    std::vector<std::vector<std::pair<int, int>>> forward_adj;  // node -> { otherNode, edge_idx }
    std::vector<std::vector<std::pair<int, int>>> reverse_adj;  // otherNode -> { node, edge_idx }
    std::vector<int> edgeCosts;
	std::vector<std::pair<int, int>> edgeFromTos; // { from, to (to == -1) for deadEnds }
	std::vector<std::vector<int>> nodeToEdgeIdxs;
    std::vector<GridType::Point> nodePoints;      // baseNodes
	std::vector<int> abstractBaseNodes;
	std::vector<int> abstractBaseEdges;
	MathStuff::Grid2D<uint32_t> edgeGrid;         // <16 dist along path of closest edge> |1=past halfway | <15 edge index>
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
