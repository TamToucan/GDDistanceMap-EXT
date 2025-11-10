#include "Routing.hpp"
#include "Router.hpp"

#include "Debug.h"

namespace Routing {

MathStuff::Grid2D<uint32_t> makeEdgeGrid(const std::vector<GridType::Edge> edges, const GridType::Grid& grid)
{
    const int rows = grid.size();
    const int cols = grid[0].size();
    MathStuff::Grid2D<uint32_t> result(cols, rows, -1);
    MathStuff::Grid2D<int> distsFromEdge(cols, rows, -1);

    // Multi-source BFS setup
    std::queue<std::tuple<GridType::Point, uint32_t, int>> q;
    for (int idx = 0; idx < edges.size(); ++idx)
    {
        const auto& edge = edges.at(idx);

        int half = 0;
        int halfCount = edge.path.size() / 2;
        for (int dist = 0; dist < edge.path.size(); ++dist)
        {
            const auto p = edge.path.at(dist);
            uint32_t val = (dist << 16) | half | idx;
            result.put(p.first, p.second, val);
            if (halfCount && --halfCount == 0) half = GridType::EDGE_HALF;
            q.emplace(p, val, 0);
            distsFromEdge.put(p.first, p.second, 0);
        }
    }

    // BFS propagation
    while (!q.empty()) {
        auto [pos, v, d] = q.front();
        q.pop();

        ++d;
        int dist = v >> 16;
        for (const auto& dir : GridType::directions8) {
            GridType::Point next = { pos.first + dir.first, pos.second + dir.second };
            if (grid[next.second][next.first] & (GridType::WALL | GridType::NODE | GridType::EDGE)) continue;

            int cell = result.get(next.first, next.second);
            if (cell == -1) {
                result.put(next.first, next.second, v);
                q.emplace(next, v, d);
                distsFromEdge.put(next.first, next.second, d);
            }
            else {
                int curDist = distsFromEdge.get(next.first, next.second);
                if (d < curDist)
                {
                    result.put(next.first, next.second, v);
                    q.emplace(next, v, d);
                    distsFromEdge.put(next.first, next.second, d);
                }
            }
        }
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////

SparseGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes, 
                            const std::vector<GridType::Point>& deadEnds, 
                            const std::vector<GridType::Edge>& baseEdges,
                            const GridType::Grid& infoGrid)
{
    SparseGraph g;
    const int numNodes = baseNodes.size();

    // Initialize adjacency lists
    g.forwardConnections.resize(numNodes);
    g.reverseConnections.resize(numNodes);
    g.nodeToEdgeIdxs.resize(numNodes);
    g.edgeCosts.reserve(baseEdges.size());
    g.edgeFromTos.reserve(baseEdges.size());
    g.nodePoints = baseNodes;

    for (size_t i = 0; i < baseEdges.size(); ++i) {
        const GridType::Edge& e = baseEdges[i];
        g.edgeCosts.push_back(e.toDeadEnd ? 0xffff : e.path.size());
        g.edgeFromTos.emplace_back(e.from, e.toDeadEnd ? -1 : e.to);

        g.forwardConnections[e.from].emplace_back(e.to, i);
        g.reverseConnections[e.from].emplace_back(e.to, i);
        if (e.toDeadEnd) {
            g.deadendConnection[e.to] = { e.from, i };
        }
        else {
            g.reverseConnections[e.to].emplace_back(e.from, i);
            g.forwardConnections[e.to].emplace_back(e.from, i);
            g.nodeToEdgeIdxs[e.from].push_back(i);
            g.nodeToEdgeIdxs[e.to].push_back(i);
        }
    }

    g.edgeGrid = makeEdgeGrid(baseEdges, infoGrid);

    return g;
}

///////////////////////////////////////////////////////////////////////////////////////////

struct SearchState {
    int node;
    int via_edge;
    int cost;
    int heuristic;

    bool operator>(const SearchState& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

///////////////////////////////////////////////////////////////////////////////////////////
// A* search core
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> bidirectionalAStarFlexible(
    const SparseGraph& g,
    std::optional<int> sourceNodeIdx,
    std::optional<int> sourceEdgeIdx,
    std::optional<int> targetNodeIdx,
    std::optional<int> targetEdgeIdx,
    const std::unordered_set<int>& allowedEdges,
    const std::unordered_set<int>& allowedNodes)
{
    // Heuristic function
    auto heuristic = [&](int node, bool isForward) {
        if (isForward) {
            if (targetNodeIdx.has_value()) {
                return std::abs(g.nodePoints[node].first - g.nodePoints[*targetNodeIdx].first)
                     + std::abs(g.nodePoints[node].second - g.nodePoints[*targetNodeIdx].second);
            } else if (targetEdgeIdx.has_value()) {
                const auto& [tgtFrom, tgtTo] = g.edgeFromTos[*targetEdgeIdx];
                auto fromDst = std::abs(g.nodePoints[node].first - g.nodePoints[tgtFrom].first)
                             + std::abs(g.nodePoints[node].second - g.nodePoints[tgtFrom].second);
                auto toDst = (tgtTo == -1 ? fromDst :
                    std::abs(g.nodePoints[node].first - g.nodePoints[tgtTo].first)
                  + std::abs(g.nodePoints[node].second - g.nodePoints[tgtTo].second));
                return std::min(fromDst, toDst);
            }
        } else {
            if (sourceNodeIdx.has_value()) {
                return std::abs(g.nodePoints[node].first - g.nodePoints[*sourceNodeIdx].first)
                     + std::abs(g.nodePoints[node].second - g.nodePoints[*sourceNodeIdx].second);
            } else if (sourceEdgeIdx.has_value()) {
                const auto& [srcFrom, srcTo] = g.edgeFromTos[*sourceEdgeIdx];
                auto fromDst = std::abs(g.nodePoints[node].first - g.nodePoints[srcFrom].first)
                             + std::abs(g.nodePoints[node].second - g.nodePoints[srcFrom].second);
                auto toDst = (srcTo == -1 ? fromDst :
                    std::abs(g.nodePoints[node].first - g.nodePoints[srcTo].first)
                  + std::abs(g.nodePoints[node].second - g.nodePoints[srcTo].second));
                return std::min(fromDst, toDst);
            }
        }
        return 0;
    };

    auto seedForward = [&](auto& q, auto& h) {
        if (sourceNodeIdx.has_value()) {
            q.push({ *sourceNodeIdx, -1, 0, h(*sourceNodeIdx, true) });
        } else if (sourceEdgeIdx.has_value()) {
            const auto& [srcFrom, srcTo] = g.edgeFromTos[*sourceEdgeIdx];
            q.push({ srcFrom, -1, 0, h(srcFrom, true) });
            if (srcTo != -1) q.push({ srcTo, -1, 0, h(srcTo, true) });
        }
    };

    auto seedBackward = [&](auto& q, auto& h) {
        if (targetNodeIdx.has_value()) {
            q.push({ *targetNodeIdx, -1, 0, h(*targetNodeIdx, false) });
        } else if (targetEdgeIdx.has_value()) {
            const auto& [tgtFrom, tgtTo] = g.edgeFromTos[*targetEdgeIdx];
            q.push({ tgtFrom, -1, 0, h(tgtFrom, false) });
            if (tgtTo != -1) q.push({ tgtTo, -1, 0, h(tgtTo, false) });
        }
    };

    // ACTUAL BIDIRECTIONAL A* IMPLEMENTATION
    std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> forwardQueue;
    std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> backwardQueue;
    
    // Data structures for tracking search progress
    std::vector<int> forwardCost(g.nodePoints.size(), INT_MAX);
    std::vector<int> backwardCost(g.nodePoints.size(), INT_MAX);
    std::vector<int> forwardParent(g.nodePoints.size(), -1);
    std::vector<int> backwardParent(g.nodePoints.size(), -1);
    std::vector<int> forwardParentEdge(g.nodePoints.size(), -1);
    std::vector<int> backwardParentEdge(g.nodePoints.size(), -1);
    
    // Initialize queues
    LOG_DEBUG("AStar: fwd");
    seedForward(forwardQueue, heuristic);
    LOG_DEBUG("AStar: bwd");
    seedBackward(backwardQueue, heuristic);
    
    // Initialize costs for seeded nodes
    if (sourceNodeIdx.has_value()) {
        LOG_DEBUG("AStar:  SRC fwdCost src:" << *sourceNodeIdx << "= 0");
        forwardCost[*sourceNodeIdx] = 0;
    } else if (sourceEdgeIdx.has_value()) {
        const auto& [srcFrom, srcTo] = g.edgeFromTos[*sourceEdgeIdx];
        LOG_DEBUG("AStar:  SRCfrom fwdCost (" << srcFrom << "->" << srcTo << ")= 0");
        forwardCost[srcFrom] = 0;
        if (srcTo != -1) forwardCost[srcTo] = 0;
    }
    
    if (targetNodeIdx.has_value()) {
        LOG_DEBUG("AStar:  TGT bwdCost tgt:" << *targetNodeIdx << "= 0");
        backwardCost[*targetNodeIdx] = 0;
    } else if (targetEdgeIdx.has_value()) {
        const auto& [tgtFrom, tgtTo] = g.edgeFromTos[*targetEdgeIdx];
        LOG_DEBUG("AStar:  TGTbwdCost (" << tgtFrom << "->" << tgtTo << ")= 0");
        backwardCost[tgtFrom] = 0;
        if (tgtTo != -1) backwardCost[tgtTo] = 0;
    }
    
    int meetingNode = -1;
    int bestCost = INT_MAX;
    
    // Main search loop
    while (!forwardQueue.empty() && !backwardQueue.empty()) {
        // Expand forward search
        if (!forwardQueue.empty()) {
            SearchState current = forwardQueue.top();
            forwardQueue.pop();
            
            // Check if we found a better path to this node
            if (current.cost > forwardCost[current.node]) continue;
            
            // Check if this node exists in backward search
            if (backwardCost[current.node] < INT_MAX) {
                int totalCost = current.cost + backwardCost[current.node];
                if (totalCost < bestCost) {
                    bestCost = totalCost;
                    meetingNode = current.node;
                }
            }
            
            // Expand neighbors
            for (const auto& [neighbor, edgeIdx] : g.forwardConnections[current.node]) {
                // Check if edge and node are allowed
                if (!allowedEdges.empty() && !allowedEdges.count(edgeIdx)) continue;
                if (!allowedNodes.empty() && !allowedNodes.count(neighbor)) continue;
                
                int newCost = current.cost + g.edgeCosts[edgeIdx];
                if (newCost < forwardCost[neighbor]) {
                    forwardCost[neighbor] = newCost;
                    forwardParent[neighbor] = current.node;
                    forwardParentEdge[neighbor] = edgeIdx;
                    int h = heuristic(neighbor, true);
                    forwardQueue.push({neighbor, edgeIdx, newCost, h});
                }
            }
        }
        
        // Expand backward search  
        if (!backwardQueue.empty()) {
            LOG_DEBUG("AStar: bwd NOT empty: " << backwardQueue.size());
            SearchState current = backwardQueue.top();
            backwardQueue.pop();
            
            if (current.cost > backwardCost[current.node]) continue;
            
            // Check if this node exists in forward search
            if (forwardCost[current.node] < INT_MAX) {
                int totalCost = current.cost + forwardCost[current.node];
                if (totalCost < bestCost) {
                    bestCost = totalCost;
                    meetingNode = current.node;
                }
            }
            
            // Expand neighbors in reverse direction
            for (const auto& [neighbor, edgeIdx] : g.reverseConnections[current.node]) {
                if (!allowedEdges.empty() && !allowedEdges.count(edgeIdx)) continue;
                if (!allowedNodes.empty() && !allowedNodes.count(neighbor)) continue;
                
                int newCost = current.cost + g.edgeCosts[edgeIdx];
                if (newCost < backwardCost[neighbor]) {
                    backwardCost[neighbor] = newCost;
                    backwardParent[neighbor] = current.node;
                    backwardParentEdge[neighbor] = edgeIdx;
                    int h = heuristic(neighbor, false);
                    backwardQueue.push({neighbor, edgeIdx, newCost, h});
                }
            }
        }
        
        // Early termination if we found a path and both queues have higher costs
        if (meetingNode != -1) {
            LOG_DEBUG("AStar: => meetingNode: " << meetingNode);
            auto forwardTop = forwardQueue.empty() ? SearchState{-1, -1, INT_MAX, 0} : forwardQueue.top();
            auto backwardTop = backwardQueue.empty() ? SearchState{-1, -1, INT_MAX, 0} : backwardQueue.top();
            
            if (forwardTop.cost + forwardTop.heuristic >= bestCost &&
                backwardTop.cost + backwardTop.heuristic >= bestCost) {
                break;
            }
        }
    }
    
    // Reconstruct path if found
    if (meetingNode == -1) return {};
    
    // Reconstruct forward path (source to meeting node)
    LOG_DEBUG("AStar: RECONSTRUCT FWD from meeting: " << meetingNode);
    std::vector<int> forwardPath;
    int node = meetingNode;
    while (forwardParent[node] != -1) {
        forwardPath.push_back(forwardParentEdge[node]);
        node = forwardParent[node];
    }
    std::reverse(forwardPath.begin(), forwardPath.end());
    LOG_DEBUG("AStar: => fwdSize: " << forwardPath.size());
    
    // Reconstruct backward path (meeting node to target)
    LOG_DEBUG("AStar: RECONSTRUCT BWD from meeting: " << meetingNode);
    std::vector<int> backwardPath;
    node = meetingNode;
    while (backwardParent[node] != -1) {
        backwardPath.push_back(backwardParentEdge[node]);
        node = backwardParent[node];
    }
    LOG_DEBUG("AStar: => bwdSize: " << forwardPath.size());
    
    // Combine paths
    forwardPath.insert(forwardPath.end(), backwardPath.begin(), backwardPath.end());
    return forwardPath;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Path conversion
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> convertEdgesToNodePath(
    const SparseGraph& routingGraph,
    const std::vector<int>& edgePath,
    std::optional<int> sourceEdgeIdx,
    std::optional<int> sourceNodeIdx)
{
    std::vector<int> nodePath;
    if (edgePath.empty()) return nodePath;
    const auto& edges = routingGraph.edgeFromTos;

    if (sourceNodeIdx.has_value()) {
        nodePath.push_back(*sourceNodeIdx);
    } else if (sourceEdgeIdx.has_value()) {
        int firstEdge = edgePath[0];
        int srcFrom = edges[*sourceEdgeIdx].first;
        int srcTo   = edges[*sourceEdgeIdx].second;

        bool isForward;
        if (edges[firstEdge].first == srcFrom || edges[firstEdge].first == srcTo) {
            isForward = true;
        }
        else if (edges[firstEdge].second == srcFrom || edges[firstEdge].second == srcTo) {
            isForward = false;
        }
        else {
            std::cout << "ERROR edge doesn't connect to source" << std::endl;
            return {};
        }

        if (isForward) {
            nodePath.push_back(edges[firstEdge].first);
            nodePath.push_back(edges[firstEdge].second);
        } else {
            nodePath.push_back(edges[firstEdge].second);
            nodePath.push_back(edges[firstEdge].first);
        }
    } else {
        std::cout << "ERROR: Must provide either sourceEdgeIdx or sourceNodeIdx" << std::endl;
        return {};
    }

    for (int i = (sourceNodeIdx ? 0 : 1); i < edgePath.size(); ++i) {
        auto currentEdge = edgePath[i];
        int prevNode = nodePath.back();

        if (edges[currentEdge].first == prevNode) {
            nodePath.push_back(edges[currentEdge].second);
        } else if (edges[currentEdge].second == prevNode) {
            nodePath.push_back(edges[currentEdge].first);
        } else {
            std::cout << "ERROR Non-contiguous edge path" << std::endl;
            break;
        }
    }

    return nodePath;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Same-zone routing methods (optimized with caching)
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> findZoneNodeToNodePath(
    const SparseGraph& routingGraph,
    Router::RouteCtx* ctx,
    const std::vector<int>& zoneBases,
    const std::vector<int>& zoneEdges,
    int sourceNodeIdx,
    int targetNodeIdx)
{
    if (ctx->lastRouteType == Router::RouteCtx::RouteType::NodeToNode &&
        ctx->routeSrcNodeIdx == sourceNodeIdx &&
        ctx->routeTgtNodeIdx == targetNodeIdx) {
        return ctx->routeNodes;
    }

    ctx->routeSrcNodeIdx = sourceNodeIdx;
    ctx->routeTgtNodeIdx = targetNodeIdx;
    ctx->lastRouteType   = Router::RouteCtx::RouteType::NodeToNode;

    std::unordered_set<int> allowedEdges(zoneEdges.begin(), zoneEdges.end());
    std::unordered_set<int> allowedNodes(zoneBases.begin(), zoneBases.end());

    auto edgePath = bidirectionalAStarFlexible(
        routingGraph, sourceNodeIdx, std::nullopt, targetNodeIdx, std::nullopt,
        allowedEdges, allowedNodes
    );

    ctx->routeNodes = convertEdgesToNodePath(routingGraph, edgePath, std::nullopt, sourceNodeIdx);
    return ctx->routeNodes;
}

std::vector<int> findZoneEdgeToNodePath(
    const SparseGraph& routingGraph,
    Router::RouteCtx* ctx,
    const std::vector<int>& zoneBases,
    const std::vector<int>& zoneEdges,
    int sourceEdgeIdx,
    int targetNodeIdx)
{
    LOG_DEBUG("FZe2np: bases: " << zoneBases.size() << " edges: " << zoneEdges.size()
        << " srcEdge: " << sourceEdgeIdx << " tgtNode: " << targetNodeIdx);
    if (ctx->lastRouteType == Router::RouteCtx::RouteType::EdgeToNode &&
        ctx->routeSrcEdgeIdx == sourceEdgeIdx &&
        ctx->routeTgtNodeIdx2 == targetNodeIdx) {
        LOG_DEBUG("FZe2np: => SAME ROUTE");
        return ctx->routeNodes;
    }

    ctx->routeSrcEdgeIdx = sourceEdgeIdx;
    ctx->routeTgtNodeIdx2 = targetNodeIdx;
    ctx->lastRouteType    = Router::RouteCtx::RouteType::EdgeToNode;

    std::unordered_set<int> allowedEdges(zoneEdges.begin(), zoneEdges.end());
    std::unordered_set<int> allowedNodes(zoneBases.begin(), zoneBases.end());

    auto edgePath = bidirectionalAStarFlexible(
        routingGraph, std::nullopt, sourceEdgeIdx, targetNodeIdx, std::nullopt,
        allowedEdges, allowedNodes
    );
    LOG_DEBUG("FZe2np: AStar conv edges#: " << edgePath.size() << " to routeNodes");
 
    ctx->routeNodes = convertEdgesToNodePath(routingGraph, edgePath, sourceEdgeIdx, std::nullopt);
    return ctx->routeNodes;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Low-level routing with allowed sets
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> findRouteWithAllowedSets(
    const SparseGraph& routingGraph,
    Router::RouteCtx* ctx,
    const std::vector<int>& allowedNodes,
    const std::vector<int>& allowedEdges,
    std::optional<int> sourceNodeIdx,
    std::optional<int> sourceEdgeIdx,
    int targetNodeIdx)
{
    std::unordered_set<int> allowedNodesSet(allowedNodes.begin(), allowedNodes.end());
    std::unordered_set<int> allowedEdgesSet(allowedEdges.begin(), allowedEdges.end());

    auto edgePath = bidirectionalAStarFlexible(
        routingGraph, sourceNodeIdx, sourceEdgeIdx, targetNodeIdx, std::nullopt,
        allowedEdgesSet, allowedNodesSet
    );

    if (sourceNodeIdx.has_value()) {
        return convertEdgesToNodePath(routingGraph, edgePath, std::nullopt, *sourceNodeIdx);
    } else {
        return convertEdgesToNodePath(routingGraph, edgePath, *sourceEdgeIdx, std::nullopt);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// Zone pathfinding
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> findZonePathBFS(const std::vector<GridType::ZoneInfo>& zones, 
                                int sourceZoneId, int targetZoneId) {
    if (sourceZoneId == targetZoneId) return {sourceZoneId};
    
    std::queue<int> q;
    std::unordered_map<int, int> parent;
    std::unordered_set<int> visited;
    
    q.push(sourceZoneId);
    parent[sourceZoneId] = -1;
    visited.insert(sourceZoneId);
    
    while (!q.empty()) {
        int current = q.front();
        q.pop();
        
        if (current == targetZoneId) {
            // Reconstruct path
            std::vector<int> path;
            for (int zone = targetZoneId; zone != -1; zone = parent[zone]) {
                path.push_back(zone);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        for (int neighbor : zones[current].adjacentZones) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }
    
    return {}; // No path found
}

///////////////////////////////////////////////////////////////////////////////////////////
// Unified routing for all scenarios
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> findRoute(
    const SparseGraph& routingGraph,
    Router::RouteCtx* ctx,
    const std::vector<GridType::ZoneInfo>& zones,
    const std::vector<GridType::AbstractEdge>& abstractEdges,
    std::optional<int> sourceNodeIdx,
    std::optional<int> sourceEdgeIdx,
    int targetNodeIdx,
    int sourceZoneId,
    int targetZoneId,  
    int zoneRelationship)
{
    // Case 1: Same zone
    if (zoneRelationship == -1) {
        LOG_DEBUG("FR: Same srcZone: " << sourceZoneId);
        const auto& zone = zones[sourceZoneId];
        
        if (sourceNodeIdx.has_value()) {
            LOG_DEBUG("FR:   Find NodeToPath");
            return findZoneNodeToNodePath(routingGraph, ctx, 
                                        zone.baseNodeIdxs, zone.baseEdgeIdxs,
                                        *sourceNodeIdx, targetNodeIdx);
        } else {
            LOG_DEBUG("FR:   Find EdgeToPath");
            return findZoneEdgeToNodePath(routingGraph, ctx,
                                        zone.baseNodeIdxs, zone.baseEdgeIdxs,
                                        *sourceEdgeIdx, targetNodeIdx);
        }
    }
    
    // Case 2: Adjacent zones
    if (zoneRelationship >= 0) {
        LOG_DEBUG("FR: Adjacent srcZ: " << sourceZoneId << " tgtZ:: " << targetZoneId);
        const auto& sourceZone = zones[sourceZoneId];
        const auto& targetZone = zones[targetZoneId];
        
        // Combine both zones
        std::vector<int> allowedNodes = sourceZone.baseNodeIdxs;
        allowedNodes.insert(allowedNodes.end(), targetZone.baseNodeIdxs.begin(), targetZone.baseNodeIdxs.end());
        
        std::vector<int> allowedEdges = sourceZone.baseEdgeIdxs;
        allowedEdges.insert(allowedEdges.end(), targetZone.baseEdgeIdxs.begin(), targetZone.baseEdgeIdxs.end());
        
        LOG_DEBUG("FR:   FindWithAllSets. nodes:" << allowedNodes.size() << " edges: " << allowedEdges.size());
        return findRouteWithAllowedSets(routingGraph, ctx, allowedNodes, allowedEdges,
                                      sourceNodeIdx, sourceEdgeIdx, targetNodeIdx);
    }
    
    // Case 3: Distant zones
    LOG_DEBUG("FR: Distant: " << sourceZoneId << " tgtZ:: " << targetZoneId);
    std::vector<int> zonePath = findZonePathBFS(zones, sourceZoneId, targetZoneId);
    if (zonePath.empty()) {
        LOG_DEBUG("FR:   => EMPTY");
        return {};
    }
    
    // Combine all zones along the path
    std::vector<int> allowedNodes, allowedEdges;
    LOG_DEBUG("FR: Make sets for " << zonePath.size() << " zones");
    for (int zoneId : zonePath) {
        const auto& zone = zones[zoneId];
        LOG_DEBUG("  Add zone: " << zoneId
            << " nodes: " << zone.baseNodeIdxs.size()
            << " edges: " << zone.baseEdgeIdxs.size());
        LOG_DEBUG_CONT("      nodes: ");
        LOG_DEBUG_FOR(auto z : zone.baseNodeIdxs, z << " ");
        LOG_DEBUG_CONT("      edges: ");
        LOG_DEBUG_FOR(auto e : zone.baseEdgeIdxs, e << " ");
        LOG_DEBUG("");
        allowedNodes.insert(allowedNodes.end(), zone.baseNodeIdxs.begin(), zone.baseNodeIdxs.end());
        allowedEdges.insert(allowedEdges.end(), zone.baseEdgeIdxs.begin(), zone.baseEdgeIdxs.end());
    }
    
    LOG_DEBUG("FR:   Made sets. nodes:" << allowedNodes.size() << " edges: " << allowedEdges.size());
    return findRouteWithAllowedSets(routingGraph, ctx, allowedNodes, allowedEdges,
                                  sourceNodeIdx, sourceEdgeIdx, targetNodeIdx);
}

} // namespace Routing
