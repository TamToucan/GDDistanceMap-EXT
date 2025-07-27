#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

#include "Routing.hpp"

#include <filesystem>

#include "GridToGraph.hpp"

namespace Routing {

MathStuff::Grid2D<uint32_t> makeEdgeGrid(const std::vector<GridType::Edge> edges, const GridType::Grid& grid)
{
    const int rows = grid.size();
    const int cols = grid[0].size();
    MathStuff::Grid2D<uint32_t> result(cols, rows, -1);
    MathStuff::Grid2D<int> distsFromEdge(cols, rows, -1);

    // Multi-source BFS setup
    std::queue<std::tuple<GridType::Point, uint32_t, int>> q;
    for (int idx=0; idx < edges.size(); ++idx)
    {
        const auto& edge = edges.at(idx);
//        if (edge.toDeadEnd) continue;

        int half = 0;
        int halfCount = edge.path.size() / 2;
        std::cerr << "EG: " << edge.from << " -> " << edge.to << " p:" << edge.path.size() << std::endl;
	    for (int dist=0; dist < edge.path.size(); ++dist)
	    {
            const auto p = edge.path.at(dist);
            uint32_t val = (dist << 16) | half | idx;
            result.put(p.first, p.second, val);
            std::cerr << "  EG: edge:" << idx << " put:" << p.first << "," << p.second
                << " d:" << dist << std::hex << " H:" << half << " v:" << val << std::dec << std::endl;
            if (halfCount && --halfCount == 0) half = GridType::EDGE_HALF;
			q.emplace(p, val, 0);
            distsFromEdge.put(p.first, p.second, 0);
	    }
    }

    // BFS propagation
    while (!q.empty()) {
        auto [pos, v, d] = q.front();
        q.pop();
        std::cerr << "Check EG: " << pos.first << "," << pos.second << " v=" << std::hex << v << std::dec << " dist:" << d << std::endl;

        ++d;
        int dist = v >> 16;
		for (const auto& dir : GridType::directions8) {
            GridToGraph::Point next = { pos.first + dir.first, pos.second + dir.second };
            if (grid[next.second][next.first] & (GridToGraph::WALL | GridToGraph::NODE|GridType::EDGE)) continue;

            int cell = result.get(next.first, next.second);
            std::cerr << "    ** " << next.first << "," << next.second << " => " << std::hex << cell << std::dec << std::endl;
            if (cell == -1) {
                result.put(next.first, next.second, v);
                std::cerr << "  dir: " << dir.first << "," << dir.second << " empty => " << next.first << "," << next.second << " d:" << d << std::endl;
                q.emplace(next, v, d);
				distsFromEdge.put(next.first, next.second, d);
            }
            else {
                int curDist = distsFromEdge.get(next.first, next.second);
	            if (d < curDist)
	            {
					result.put(next.first, next.second, v);
					q.emplace(next, v, d);
                    std::cerr << "  dir: " << dir.first << "," << dir.second << " d<cur " << d << " < " << curDist << std::endl;
					distsFromEdge.put(next.first, next.second, d);
	            }
            }
		}
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////

SparseGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes, const std::vector<GridType::Edge>& baseEdges,
    const GridType::Grid& infoGrid)
{
    SparseGraph g;
    const int numNodes = baseNodes.size();

    // Initialize adjacency lists
    g.forward_adj.resize(numNodes);
    g.reverse_adj.resize(numNodes);
    g.nodeToEdgeIdxs.resize(numNodes);
    g.edgeCosts.reserve(baseEdges.size());
    g.edgeFromTos.reserve(baseEdges.size());
    g.nodePoints = baseNodes;

    for (size_t i = 0; i < baseEdges.size(); ++i) {
        const GridType::Edge& e = baseEdges[i];
        //if (e.toDeadEnd) continue;

        g.edgeCosts.push_back(e.toDeadEnd ? 0xffff : e.path.size());
        g.edgeFromTos.emplace_back(e.from, e.toDeadEnd ? -1 : e.to);

		g.forward_adj[e.from].emplace_back(e.to, i);
        g.reverse_adj[e.from].emplace_back(e.to, i);
        if (!e.toDeadEnd) {
			g.reverse_adj[e.to].emplace_back(e.from, i);
			g.forward_adj[e.to].emplace_back(e.from, i);

            g.nodeToEdgeIdxs[e.from].push_back(i);
            g.nodeToEdgeIdxs[e.to] .push_back(i);
        }
    }

    g.edgeGrid = makeEdgeGrid(baseEdges, infoGrid);

    std::cerr << "EDGE GRID" << std::endl;
    std::cerr << "     ";
	for (int x = 0; x < g.edgeGrid.width(); ++x)
	{
        std::cerr << (x < !0 ? " " : "") << x << "      ";
	}
    std::cerr << std::endl;
    for (int y=0; y < g.edgeGrid.height(); ++y)
    {
        std::cerr << (y < 10 ? " " : "") << y << "  ";
	    for (int x=0; x < g.edgeGrid.width(); ++x)
	    {
            uint32_t cell = g.edgeGrid.get(x, y);
            if (cell == 0xffffffff)
            {
                std::cerr << "  :    ";
                continue;
            }
            int dist = cell >> 16;
            int edg = cell & GridType::EDGE_MASK;
            std::cerr << (dist < 10 ? " " : "") << dist << ":" << (edg < 10 ? " " : "") << edg << "  ";
	    }
        std::cerr << std::endl;
    }
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

std::vector<int> bidirectionalAStar(const SparseGraph& g, int sourceEdgeIdx, int targetEdgeIdx,
    const std::unordered_set<int>& allowedEdges, const std::unordered_set<int>& allowedNodes)
{
    const auto& [srcFrom, srcTo] = g.edgeFromTos[sourceEdgeIdx];
    const auto& [tgtFrom, tgtTo] = g.edgeFromTos[targetEdgeIdx];

    std::cerr << "srcEdge: " << sourceEdgeIdx << " from: " << srcFrom << " -> " << srcTo << std::endl;
    std::cerr << "dstEdge: " << targetEdgeIdx << " from: " << tgtFrom << " -> " << tgtTo << std::endl;

    std::cerr << "  Allowed Edges:";
    for (const int e : allowedEdges)
    {
        std::cerr << "  " << e << "("<<g.edgeFromTos[e].first<<"->"<<g.edgeFromTos[e].second<<") ";
    }
    std::cerr << std::endl;
    std::cerr << "  Allowed Nodes:";
    for (const int n : allowedNodes)
    {
        std::cerr << "  " << n;
    }
    std::cerr << std::endl;

    // Heuristic helper function
    auto heuristic = [&](int node, bool isForward) {
        if (isForward) {
            auto fromDst = std::abs(g.nodePoints[node].first - g.nodePoints[tgtFrom].first)
                + std::abs(g.nodePoints[node].second - g.nodePoints[tgtFrom].second);
            auto toDst = tgtTo == -1 ? fromDst
                : std::abs(g.nodePoints[node].first - g.nodePoints[tgtTo].first)
                + std::abs(g.nodePoints[node].second - g.nodePoints[tgtTo].second);
            auto ret = std::min(fromDst, toDst);
#if 0
            auto ret = std::min(
                std::abs(g.nodePoints[node].first - g.nodePoints[tgtFrom].first) +
                std::abs(g.nodePoints[node].second - g.nodePoints[tgtFrom].second),
                std::abs(g.nodePoints[node].first - g.nodePoints[tgtTo].first) +
                std::abs(g.nodePoints[node].second - g.nodePoints[tgtTo].second)
            );
#endif
            std::cerr << "   FWD HEUR node:" << node << " ret: " << ret << std::endl;
            return ret;
        }
		auto fromDst = std::abs(g.nodePoints[node].first - g.nodePoints[srcFrom].first)
			+ std::abs(g.nodePoints[node].second - g.nodePoints[srcFrom].second);
		auto toDst = srcTo == -1 ? fromDst
			: std::abs(g.nodePoints[node].first - g.nodePoints[srcTo].first)
			+ std::abs(g.nodePoints[node].second - g.nodePoints[srcTo].second);
		auto ret = std::min(fromDst, toDst);
#if 0
        auto ret = std::min(
            std::abs(g.nodePoints[node].first - g.nodePoints[srcFrom].first) +
            std::abs(g.nodePoints[node].second - g.nodePoints[srcFrom].second),
            std::abs(g.nodePoints[node].first - g.nodePoints[srcTo].first) +
            std::abs(g.nodePoints[node].second - g.nodePoints[srcTo].second)
        );
#endif
		std::cerr << "   BCK HEUR node:" << node << " ret: " << ret << std::endl;
		return ret;
		};

    // Initialization
    std::priority_queue<SearchState, std::vector<SearchState>, std::greater<>> forward_q, backward_q;
    std::vector<int> forward_cost(g.forward_adj.size(), INT_MAX);
    std::vector<int> backward_cost(g.forward_adj.size(), INT_MAX);
    std::vector<int> forward_edge(g.forward_adj.size(), -1);
    std::vector<int> backward_edge(g.forward_adj.size(), -1);

    // Seed both searches
    forward_q.push({ srcFrom, sourceEdgeIdx, 0, heuristic(srcFrom, true) });
    if (srcTo != -1) {
        forward_q.push({ srcTo,sourceEdgeIdx, 0, heuristic(srcTo, true) });
    }
    else {
        const auto& connections = g.nodeToEdgeIdxs[srcFrom];
        std::cerr << "SRC TO is deadEnd add Conns: " << connections.size() << std::endl;
        for (auto edgeIdx : connections) {
            const auto [f, t] = g.edgeFromTos[edgeIdx];
			std::cerr << "   Check edge: " << edgeIdx << std::endl;
            if (f != srcFrom) {
                std::cerr << "     ADD from fwd con: " << f << std::endl;
                forward_q.push({ f, edgeIdx, 0, heuristic(f, true) });
            }
            if (t != srcFrom && t != -1) {
                std::cerr << "     ADD to fwd con: " << t << std::endl;
                forward_q.push({ t, edgeIdx, 0, heuristic(t, true) });
            }
        }
    }

    backward_q.push({ tgtFrom, targetEdgeIdx, 0, heuristic(tgtFrom, false) });
    if (tgtTo != -1) {
        backward_q.push({ tgtTo, targetEdgeIdx, 0, heuristic(tgtTo, false) });
    }
    else {
		const auto& connections = g.nodeToEdgeIdxs[tgtFrom];
		std::cerr << "DST is deadEnd add Conns: " << connections.size() << std::endl;
		for (auto edgeIdx : connections) {
			const auto [f, t] = g.edgeFromTos[edgeIdx];
			std::cerr << "   Check edge: " << edgeIdx << " " << f <<" -> " << t << std::endl;
            if (f != tgtFrom) {
                std::cerr << "     ADD to back con: " << f << std::endl;
				backward_q.push({ f, edgeIdx, 0, heuristic(f, false) });
            }
            if (t != tgtFrom && t != -1) {
                std::cerr << "     ADD to back con: " << t << std::endl;
				backward_q.push({ t, edgeIdx, 0, heuristic(t, false) });
            }
		}
    }

    int meeting_node = -1;
    bool found_path = false;

    while (!forward_q.empty() && !backward_q.empty() && !found_path) {
        // Expand forward search
        if (!forward_q.empty()) {
            std::cerr << "  FORWARD" << std::endl;
            auto f = forward_q.top();
            forward_q.pop();

            std::cerr << "  fwdCost:" << forward_cost[f.node] << " node: " << f.node << " cost:" << f.cost << " via: " << f.via_edge << std::endl;

            if (f.cost >= forward_cost[f.node]) {
                std::cerr << "   cost too much" << std::endl;
                continue;
            }
            if (!allowedNodes.count(f.node)) {
                std::cerr << "   Node not allowed" << std::endl;
                continue;
            }
            std::cerr << "  store: " << f.node << " = cost: " << f.cost << " via: " << f.via_edge << std::endl;
				
            forward_cost[f.node] = f.cost;
            forward_edge[f.node] = f.via_edge;

            // Check for intersection
            if (backward_cost[f.node] != INT_MAX) {
                meeting_node = f.node;
                found_path = true;
                std::cerr << "  Found path. meet: " << meeting_node << std::endl;
                break;
            }
            // Expand neighbors
            for (const auto& [neighbor, edge] : g.forward_adj[f.node]) {
                std::cerr << "    neighbor: " << neighbor << " edge:" << edge << std::endl;
                if (!allowedEdges.count(edge)) continue;
                int new_cost = f.cost + g.edgeCosts[edge];
                std::cerr << "      allowed: newCost: " << new_cost << " fCost:" << f.cost << " + " << g.edgeCosts[edge] << std::endl;
                if (new_cost < forward_cost[neighbor]) {
                    forward_q.push({ neighbor, edge, new_cost, heuristic(neighbor, true) });
                    std::cerr << "      => cheaper. Q up neigh: " << neighbor << " edge: " << edge << std::endl;
                }
            }
        }

        // Expand backward search
        if (!backward_q.empty()) {
            std::cerr << "  BACK" << std::endl;
            auto b = backward_q.top();
            backward_q.pop();

            std::cerr << "  bckCost:" << backward_cost[b.node] << " node: " << b.node << " cost:" << b.cost << " via: " << b.via_edge << std::endl;

            if (b.cost >= backward_cost[b.node]) continue;
            if (!allowedNodes.count(b.node)) continue;
            std::cerr << "  store: " << b.node << " = cost: " << b.cost << " via: " << b.via_edge << std::endl;

            backward_cost[b.node] = b.cost;
            backward_edge[b.node] = b.via_edge;

            // Check for immediate intersection
            if (forward_cost[b.node] != INT_MAX) {
                meeting_node = b.node;
                found_path = true;
                std::cerr << "  BACK found path. meet: " << meeting_node << std::endl;
                break;
            }

            // Expand neighbors (using reverse adjacency)
            for (const auto& [neighbor, edge] : g.reverse_adj[b.node]) {
                if (!allowedEdges.count(edge)) continue;
                int new_cost = b.cost + g.edgeCosts[edge];
                std::cerr << "      allowed: newCost: " << new_cost << " fCost:" << b.cost << " + " << g.edgeCosts[edge] << std::endl;
                if (new_cost < backward_cost[neighbor]) {
                    backward_q.push({
                        neighbor,
                        edge,
                        new_cost,
                        heuristic(neighbor, false)
                        });
                    std::cerr << "      => cheaper. BCK Q up neigh: " << neighbor << " edge: " << edge << std::endl;
                }
            }
        }
    }
    if (!found_path) {
        std::cerr << "=> NO PATH" << std::endl;
        return {};
    }

	std::cerr << "=> MEETING NODE: " << meeting_node << std::endl;
	std::cerr << "=> FWD PATH: " << forward_edge.size() << std::endl;
	for (int i = 0; i < forward_edge.size(); ++i) {
		if (forward_edge[i] != -1) std::cerr << "  fwd_edge[" << i << "] = " << forward_edge[i] << std::endl;
	}
	std::cerr << "=> BCK PATH: " << backward_edge.size() << std::endl;
	for (int i = 0; i < backward_edge.size(); ++i) {
		if (backward_edge[i] != -1) std::cerr << "  bck_edge[" << i << "] = " << backward_edge[i] << std::endl;
	}

    // Reconstruct path
    if (meeting_node == -1) return {};

    std::vector<int> path;
    int current = meeting_node;
    // Forward walk (meeting_node -> target)
    //    Uses forward_edge[] to trace toward the target
    current = meeting_node;
    while (current != -1 && forward_edge[current] != -1) {
        int idx = forward_edge[current];
		std::cerr << "  FWD: " << idx << std::endl;
        path.push_back(idx);
        const auto& edge = g.edgeFromTos[idx];
		std::cerr << "      => edge " << edge.first << " -> " << edge.second << std::endl;
        current = (edge.first == current) ? edge.second : edge.first;
    }
    std::reverse(path.begin(), path.end());

    // Backward walk (meeting_node -> source)
    //    Uses backward_edge[] to trace toward the source
    while (current != -1 && backward_edge[current] != -1) {
        int idx = backward_edge[current];
		std::cerr << "  BCK: " << idx << std::endl;
        path.push_back(idx);
        const auto& edge = g.edgeFromTos[idx];
		std::cerr << "      => edge " << edge.first << " -> " << edge.second << std::endl;
        current = (edge.first == current) ? edge.second : edge.first;
    }

    std::cerr << "=> PATH " << path.size() << ":  ";
    for (const auto p : path) std::cerr << p << "  ";
    std::cerr << std::endl;

    return path;
}

std::vector<int> convertEdgesToNodePath(const SparseGraph& routingGraph, const std::vector<int>& edgePath, int sourceEdgeIdx)
{
    std::vector<int> nodePath;
    if (edgePath.empty()) return nodePath;
    const auto& edges = routingGraph.edgeFromTos;

    // 1. Determine direction of first edge relative to source
    int firstEdge = edgePath[0];
    int srcFrom = edges[sourceEdgeIdx].first;
    int srcTo = edges[sourceEdgeIdx].second;

    bool isForward;
    if (edges[firstEdge].first == srcFrom || edges[firstEdge].first == srcTo) {
        isForward = true;
    }
    else if (edges[firstEdge].second == srcFrom || edges[firstEdge].second == srcTo) {
        isForward = false;
    }
    else {
        std::cout << "ERROR edge doesn't connect to source: 1st:" << firstEdge
    	<< " (" << edges[firstEdge].first << "->" << edges[firstEdge].second << ")"
    	<< " src: " << sourceEdgeIdx << " (" << srcFrom << "->" << srcTo << ")" << std::endl;
    }

    // 2. Add first edge's nodes
    if (isForward) {
        nodePath.push_back(edges[firstEdge].first);
        nodePath.push_back(edges[firstEdge].second);
        std::cerr << "FWD:  Add " << nodePath.front() << " " << nodePath.back() << std::endl;
    }
    else {
        nodePath.push_back(edges[firstEdge].second);
        nodePath.push_back(edges[firstEdge].first);
        std::cerr << "BCK:  Add " << nodePath.front() << " " << nodePath.back() << std::endl;
    }

    // 3. Process subsequent edges
    for (int i = 1; i < edgePath.size(); ++i) {
		auto currentEdge = edgePath[i];
        int prevNode = nodePath.back();

        if (edges[currentEdge].first == prevNode) {
            nodePath.push_back(edges[currentEdge].second);
			std::cerr << "   Addto " <<  nodePath.back() << std::endl;
        }
        else if (edges[currentEdge].second == prevNode) {
            nodePath.push_back(edges[currentEdge].first);
			std::cerr << "   AddFrom " <<  nodePath.back() << std::endl;
        }
        else {
			std::cout << "ERROR Non-contiguous edge path" << std::endl;
			std::cerr << "ERROR Non-contiguous edge path" << std::endl;
        }
    }

    return nodePath;
}

// Entry point function
std::vector<int> findZonePath(const SparseGraph routingGraph,
    const std::vector<int>& zoneBases,
    const std::vector<int>& zoneEdges,
    int sourceEdgeIdx,
    int targetEdgeIdx)
{
    std::cerr << "## Find ZonePathRoute fromEdge: " << sourceEdgeIdx << " toEdge: " << targetEdgeIdx << std::endl;
    std::cerr << "   Using " << zoneEdges.size() << " zone edges and " << zoneBases.size() << " bases" << std::endl;
    std::cerr << "   Edges:";
    for (auto z : zoneEdges) std::cerr << "  " << z;
    std::cerr << "\n   Bases:";
    for (auto b : zoneBases) std::cerr << "  " << b;
    std::cerr << std::endl;

    // Create allowed sets
    std::unordered_set<int> allowedEdges(zoneEdges.begin(), zoneEdges.end());
    std::unordered_set<int> allowedNodes(zoneBases.begin(), zoneBases.end());

    // Run bidirectional A*
    const auto edgePath = bidirectionalAStar(
        routingGraph,
        sourceEdgeIdx,
        targetEdgeIdx,
        allowedEdges,
        allowedNodes
    );
    return convertEdgesToNodePath(routingGraph, edgePath, sourceEdgeIdx);
}

}