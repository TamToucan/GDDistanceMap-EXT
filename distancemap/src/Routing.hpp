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

namespace Routing {

// SparseGraph structure with hierarchical information
struct SparseGraph {
    // Base graph adjacency (not deadEnds)
    std::vector<std::vector<std::pair<int, int>>> forwardConnections;  // node -> [ { otherNode, edge_idx },... ]
    std::vector<std::vector<std::pair<int, int>>> reverseConnections;  // otherNode -> [ { node, edge_idx },... ]
    std::unordered_map<int, std::pair<int, int>>  deadendConnection;   // deadNode -> { node , edge_idx }
    
    // Edge-node mappings
    std::vector<std::vector<int>> nodeToEdgeIdxs;
    std::vector<int> edgeCosts;
    std::vector<std::pair<int, int>> edgeFromTos; // { from, to (to == -1) for deadEnds }
    
    // Spatial information
    std::vector<GridType::Point> nodePoints;
    MathStuff::Grid2D<uint32_t> edgeGrid; // <16 dst along path of closest edge> |1=past halfway | <15 edge idx>
    
    // Hierarchical information (optional)
    std::vector<int> nodeToZone;                    // baseNode -> zoneId
    std::vector<std::vector<int>> zoneToNodes;      // zoneId -> [baseNodes]
    std::vector<std::vector<int>> zoneToEdges;      // zoneId -> [baseEdges]

    int getEdgeFromNodes(int n1, int n2) const {
        const auto& connections = forwardConnections[n1];
        for (const auto& nodeEdge : connections) {
            if (nodeEdge.first == n2) {
                return nodeEdge.second;
            }
        }
        return -1;
    }
    
    int getEdgeFromDead(int deadIdx) const {
        auto it = deadendConnection.find(deadIdx);
        return (it != deadendConnection.end()) ? it->second.second : -1;
    }
    
    // Helper methods
    int getNodeZone(int nodeIdx) const {
        return nodeToZone.empty() ? -1 : 
               (nodeIdx < nodeToZone.size() ? nodeToZone[nodeIdx] : -1);
    }
    
    const std::vector<int>& getZoneNodes(int zoneId) const {
        static const std::vector<int> empty;
        return zoneToNodes.empty() || zoneId >= zoneToNodes.size() ? empty : zoneToNodes[zoneId];
    }
    
    const std::vector<int>& getZoneEdges(int zoneId) const {
        static const std::vector<int> empty;
        return zoneToEdges.empty() || zoneId >= zoneToEdges.size() ? empty : zoneToEdges[zoneId];
    }
};

// Core graph building functions
MathStuff::Grid2D<uint32_t> makeEdgeGrid(const std::vector<GridType::Edge> edges, const GridType::Grid& grid);
SparseGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes, 
                            const std::vector<GridType::Point>& deadEnds, 
                            const std::vector<GridType::Edge>& baseEdges,
                            const GridType::Grid& infoGrid);

// Same-zone routing methods (optimized with caching)
std::vector<int> findZoneNodeToNodePath(
    const SparseGraph& routingGraph,
    Router::RouteCtx* ctx,
    const std::vector<int>& zoneBases,
    const std::vector<int>& zoneEdges,
    int sourceNodeIdx,
    int targetNodeIdx);

std::vector<int> findZoneEdgeToNodePath(
    const SparseGraph& routingGraph,
    Router::RouteCtx* ctx,
    const std::vector<int>& zoneBases,
    const std::vector<int>& zoneEdges,
    int sourceEdgeIdx,
    int targetNodeIdx);

// Unified routing interface for all scenarios
std::vector<int> findRoute(
    const SparseGraph& routingGraph,
    Router::RouteCtx* ctx,
    const std::vector<GridType::ZoneInfo>& zones,
    const std::vector<GridType::AbstractEdge>& abstractEdges,
    std::optional<int> sourceNodeIdx,
    std::optional<int> sourceEdgeIdx,
    int targetNodeIdx,
    int sourceZoneId,      // Precomputed source zone
    int targetZoneId,      // Precomputed target zone  
    int zoneRelationship); // -1 = same zone, >=0 = adjacent zone index, -2 = distant

} // namespace Routing

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
