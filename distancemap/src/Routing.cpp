#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <algorithm>

#include "Routing.hpp"
#include "GridToGraph.hpp"


namespace Routing {

SparseGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes, const std::vector<GridType::Edge>& baseEdges,
    const GridType::Grid& infoGrid)
{
    SparseGraph g;
    const int numNodes = baseNodes.size();

    // Initialize adjacency lists
    g.forward_adj.resize(numNodes);
    g.reverse_adj.resize(numNodes);
    g.edgeCosts.reserve(baseEdges.size());
    g.edgeFromTos.reserve(baseEdges.size());
    g.nodePoints = baseNodes;

    for (size_t i = 0; i < baseEdges.size(); ++i) {
        const GridType::Edge& e = baseEdges[i];
        if (e.toDeadEnd) continue;

        g.edgeCosts.push_back(e.path.size());
        g.edgeFromTos.emplace_back(e.from, e.to);

        // Forward connection
        g.forward_adj[e.from].emplace_back(e.to, i);
        g.reverse_adj[e.to].emplace_back(e.from, i);

        // Reverse connection (unless dead end)
        g.forward_adj[e.to].emplace_back(e.from, i);
        g.reverse_adj[e.from].emplace_back(e.to, i);
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

    // Heuristic helper function
    auto heuristic = [&](int node, bool isForward) {
        if (isForward) {
            return std::min(
                std::abs(g.nodePoints[node].first - g.nodePoints[tgtFrom].first) +
                std::abs(g.nodePoints[node].second - g.nodePoints[tgtFrom].second),
                std::abs(g.nodePoints[node].first - g.nodePoints[tgtTo].first) +
                std::abs(g.nodePoints[node].second - g.nodePoints[tgtTo].second)
            );
        }
        return std::min(
            std::abs(g.nodePoints[node].first - g.nodePoints[srcFrom].first) +
            std::abs(g.nodePoints[node].second - g.nodePoints[srcFrom].second),
            std::abs(g.nodePoints[node].first - g.nodePoints[srcTo].first) +
            std::abs(g.nodePoints[node].second - g.nodePoints[srcTo].second)
        );
        };

    // Initialization
    std::priority_queue<SearchState, std::vector<SearchState>, std::greater<>> forward_q, backward_q;
    std::vector<int> forward_cost(g.forward_adj.size(), INT_MAX);
    std::vector<int> backward_cost(g.forward_adj.size(), INT_MAX);
    std::vector<int> forward_edge(g.forward_adj.size(), -1);
    std::vector<int> backward_edge(g.forward_adj.size(), -1);

    // Seed both searches
    forward_q.push({ srcFrom, -1, 0, heuristic(srcFrom, true) });
    forward_q.push({ srcTo, -1, 0, heuristic(srcTo, true) });
    backward_q.push({ tgtFrom, -1, 0, heuristic(tgtFrom, false) });
    backward_q.push({ tgtTo, -1, 0, heuristic(tgtTo, false) });

    int meeting_node = -1;
    bool found_path = false;

    while (!forward_q.empty() && !backward_q.empty() && !found_path) {
        // Expand forward search
        if (!forward_q.empty()) {
            std::cerr << "  FORWARD" << std::endl;
            auto f = forward_q.top();
            forward_q.pop();

            if (f.cost >= forward_cost[f.node]) continue;
            if (!allowedNodes.count(f.node)) continue;

            forward_cost[f.node] = f.cost;
            forward_edge[f.node] = f.via_edge;

            // Check for intersection
            if (backward_cost[f.node] != INT_MAX) {
                meeting_node = f.node;
                found_path = true;
                break;
            }
            // Expand neighbors
            for (const auto& [neighbor, edge] : g.forward_adj[f.node]) {
                if (!allowedEdges.count(edge)) continue;
                int new_cost = f.cost + g.edgeCosts[edge];
                if (new_cost < forward_cost[neighbor]) {
                    forward_q.push({ neighbor, edge, new_cost, heuristic(neighbor, true) });
                }
            }
        }

        // Expand backward search
        if (!backward_q.empty()) {
            std::cerr << "  BACK" << std::endl;
            auto b = backward_q.top();
            backward_q.pop();

            if (b.cost >= backward_cost[b.node]) continue;
            if (!allowedNodes.count(b.node)) continue;

            backward_cost[b.node] = b.cost;
            backward_edge[b.node] = b.via_edge;

            // Check for immediate intersection
            if (forward_cost[b.node] != INT_MAX) {
                meeting_node = b.node;
                found_path = true;
                break;
            }

            // Expand neighbors (using reverse adjacency)
            for (const auto& [neighbor, edge] : g.reverse_adj[b.node]) {
                if (!allowedEdges.count(edge)) continue;
                int new_cost = b.cost + g.edgeCosts[edge];
                if (new_cost < backward_cost[neighbor]) {
                    backward_q.push({
                        neighbor,
                        edge,
                        new_cost,
                        heuristic(neighbor, false)
                        });
                }
            }
        }
    }
    if (!found_path) {
        std::cerr << "************ NO PATH" << std::endl;
        return {};
    }

    // Reconstruct path
    if (meeting_node == -1) return {};

    std::vector<int> path;

    // Backward reconstruction from meeting node
    int current = meeting_node;
    while (backward_edge[current] != -1) {
        std::cerr << "    Store BACK: cur: " << current << " => " << backward_edge[current]  << std::endl;
        path.push_back(backward_edge[current]);
        const auto& edge = g.edgeFromTos[backward_edge[current]];
        current = (edge.first == current) ? edge.second : edge.first;
    }
    std::reverse(path.begin(), path.end());

    // Forward reconstruction to meeting node
    current = meeting_node;
    while (forward_edge[current] != -1) {
        const auto& edge = g.edgeFromTos[forward_edge[current]];
        std::cerr << "    Store FOR: cur: " << current << " => " << forward_edge[current]  << std::endl;
        path.push_back(forward_edge[current]);
        current = (edge.first == current) ? edge.second : edge.first;
    }
    std::cerr << "=> PATH " << path.size() << ":  ";
    for (const auto p : path) std::cerr << p << "  ";
    std::cerr << std::endl;

    return path;
}

// Entry point function
std::vector<int> findZonePath(const SparseGraph routingGraph,
    const std::vector<int>& zoneBases,
    const std::vector<int>& zoneEdges,
    int sourceEdgeIdx,
    int targetEdgeIdx)
{
    std::cerr << "## Route from: " << sourceEdgeIdx << " to " << targetEdgeIdx << std::endl;
    // Create allowed sets
    std::unordered_set<int> allowedEdges(zoneEdges.begin(), zoneEdges.end());
    std::unordered_set<int> allowedNodes(zoneBases.begin(), zoneBases.end());

    // Run bidirectional A*
    return bidirectionalAStar(
        routingGraph,
        sourceEdgeIdx,
        targetEdgeIdx,
        allowedEdges,
        allowedNodes
    );
}

}