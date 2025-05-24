/*
 * AbstractMST.cpp
 *
 *  Created on: 22 Mar 2025
 *      Author: tam
 */

#include "AbstractMST.hpp"
#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <unordered_map>

#include "gdextension_interface.h"
#include "GridTypes.hpp"

using namespace GridType;

AbstractMST::AbstractMST() {

}

AbstractMST::~AbstractMST() {
}

// Union-Find (Disjoint Set) structure for Kruskal's algorithm.
struct UnionFind {
    std::vector<int> parent;
    UnionFind(int n) : parent(n) {
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }
    int find(int x) {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    }
    void unite(int x, int y) {
        parent[find(x)] = find(y);
    }
};

// Dijkstra's algorithm: given a starting base node, compute the shortest path to all base nodes.
// Returns the distance vector, predecessor vector, and usedEdge vector (which stores the edge index and direction used).
void runDijkstra(int startIdx, const BaseGraph& graph,
                 std::vector<int>& costs,
				 std::vector<int>& prevNodes,
                 std::vector<std::pair<int, bool>>& usedEdge)
{
    int numNodes = static_cast<int>(graph.size());
    costs.assign(numNodes, std::numeric_limits<int>::max());
    prevNodes.assign(numNodes, -1);
    usedEdge.assign(numNodes, std::make_pair(-1, true));

    using CostNode = std::pair<int, int>;  // (cost, base node)
    std::priority_queue<CostNode, std::vector<CostNode>, std::greater<CostNode>> pq;

    costs[startIdx] = 0;
    pq.push({0, startIdx});

    while (!pq.empty()) {
        auto [cost, nodeIdx] = pq.top();
        pq.pop();
		// if cost is already outdated, skip
        if (cost > costs[nodeIdx]) continue;

		// check all the edges from this node
        for (const BaseGraphInfo& adj : graph[nodeIdx]) {
            int neighborIdx = adj.neighbor;
            int newCost = cost + adj.cost;
            if (newCost < costs[neighborIdx]) {
                costs[neighborIdx] = newCost;
                prevNodes[neighborIdx] = nodeIdx;
                usedEdge[neighborIdx] = { adj.edgeIndex, adj.forward };
                pq.push({ newCost, neighborIdx });
            }
        }
    }
}

// Reconstruct the path (as a sequence of Points) between two base nodes given the Dijkstra predecessors.
Path reconstructPath(int startIdx, int targetIdx, const std::vector<int>& prevNodes,
                     const std::vector<std::pair<int, bool>>& usedEdge,
                     const std::vector<Edge>& edges)
{
    std::vector<std::pair<int, bool>> edgeSequence;
    int curIdx = targetIdx;
    while (curIdx != startIdx) {
        edgeSequence.push_back(usedEdge[curIdx]);
        curIdx = prevNodes[curIdx];
    }
    std::reverse(edgeSequence.begin(), edgeSequence.end());

    Path fullPath;
    bool firstSegment = true;
    for (auto [edgeIdx, forward] : edgeSequence) {
        const Path &edgePath = edges[edgeIdx].path;
        Path segment = edgePath; // Copy the segment.
        if (!forward)
            std::reverse(segment.begin(), segment.end());
        // Remove duplicate junction point if not the first segment.
        if (!firstSegment && !segment.empty())
            segment.erase(segment.begin());
        fullPath.insert(fullPath.end(), segment.begin(), segment.end());
        firstSegment = false;
    }
    return fullPath;
}


// Compute candidate AbstractEdges between all pairs of AbstractNodes.
// For each abstract node pair (i, j) with i < j, run Dijkstra's from abstractNodes[i].baseCenterNode
// to determine the shortest path to abstractNodes[j].baseCenterNode.
void computeCandidateEdges(const BaseGraph& inBaseGraph,
						   const std::vector<Edge>& edges,
                           const std::vector<Point>& baseNodes,
                           const std::vector<AbstractNode>& abstractNodes,
                           std::vector<AbstractEdge>& candidates)
{
    int numBaseNodes = static_cast<int>(baseNodes.size());
    BaseGraph gened;
	if (inBaseGraph.empty()) {
		gened = buildBaseGraph(edges, numBaseNodes);
	}
	const BaseGraph& baseGraph = (inBaseGraph.empty()) ? gened : inBaseGraph;

    int numAbstractNodes = static_cast<int>(abstractNodes.size());
    candidates.clear();
    std::cerr << "##MST: computeCandidateEdges: " << numAbstractNodes << " abstract nodes." << std::endl;

    if (numAbstractNodes < 2) { // No pairs to form edges
        if (numAbstractNodes == 1 && abstractNodes[0].baseNodes.empty()) {
             // Handle case with one abstract node that has no base nodes if necessary, though typically MST isn't for single nodes.
        }
        std::cerr << "   => Not enough abstract nodes for MST candidates." << std::endl;
        return;
    }
    
    std::vector<std::future<std::vector<AbstractEdge>>> futures;
    futures.reserve(numAbstractNodes);

    // Launch a task for each abstract node 'i' to find paths to subsequent nodes 'j'
    for (int i = 0; i < numAbstractNodes; ++i) {
        futures.emplace_back(std::async(std::launch::async, 
            [i, &baseGraph, &edges, &abstractNodes, numAbstractNodes]() {
            std::vector<AbstractEdge> local_candidates;
            int startBase = abstractNodes[i].baseCenterNode;

            if (startBase < 0 || startBase >= baseGraph.size()) {
                // Invalid startBase, perhaps an abstract node with no valid baseCenterNode
                // Or baseGraph might be smaller than expected if abstractNodes[i].baseCenterNode is an invalid index.
                // This check depends on how baseCenterNode is guaranteed to be valid.
                // For safety, one might add error logging or skip.
                return local_candidates; // Return empty if startBase is problematic
            }

            std::vector<int> costs;
            std::vector<int> prevNodes;
            std::vector<std::pair<int, bool>> usedEdge;
            runDijkstra(startBase, baseGraph, costs, prevNodes, usedEdge);

            for (int j = i + 1; j < numAbstractNodes; ++j) {
                int targetBase = abstractNodes[j].baseCenterNode;
                if (targetBase < 0 || targetBase >= costs.size() || costs[targetBase] == std::numeric_limits<int>::max()) {
                    continue; // Not reachable or invalid targetBase
                }
                AbstractEdge ce;
                ce.from = i;
                ce.to = j;
                ce.path = reconstructPath(startBase, targetBase, prevNodes, usedEdge, edges);
                if (!ce.path.empty()) { // Ensure path reconstruction was successful
                    local_candidates.push_back(ce);
                }
            }
            return local_candidates;
        }));
    }

    // Collect results from all futures
    for (auto& fut : futures) {
        std::vector<AbstractEdge> local_candidates = fut.get();
        candidates.insert(candidates.end(), local_candidates.begin(), local_candidates.end());
    }

	std::cerr << "   => " << candidates.size() << " candidates found." << std::endl;
}

// Run Kruskal's algorithm on the candidate edges to build the MST over abstract nodes.
std::vector<AbstractEdge> buildMST(const std::vector<AbstractEdge>& candidates, int numAbstractNodes)
{
    std::vector<AbstractEdge> mstEdges;
    // Start with all candidate edges and sort them by weight.
    std::vector<AbstractEdge> sortedEdges = candidates;
    std::sort(sortedEdges.begin(), sortedEdges.end(),
              [](const AbstractEdge &a, const AbstractEdge &b) {
                  return a.path.size() < b.path.size();
              });

    // Use a union-find structure to detect cycles.
    UnionFind uf(numAbstractNodes);

	std::cerr << "MST: BUILD from " << sortedEdges.size() << " candidates" << std::endl;
    for (const AbstractEdge& ce : sortedEdges) {
        int setA = uf.find(ce.from);
        int setB = uf.find(ce.to);
        if (setA != setB) {
            uf.unite(setA, setB);
            mstEdges.push_back(ce);
            // Stop if we have n-1 edges.
            if (mstEdges.size() == static_cast<size_t>(numAbstractNodes - 1))
                break;
        }
    }
	std::cerr << "   ADDed MST EDGEs: " << mstEdges.size() << std::endl;
	return mstEdges;
}

//
// Idex since can have multiple Edges with same From/To (but diff path), to keep
// the Edge which has the shorter path. But for 3200 Edges only 90 "duplicates"
// which does not seem a large enough saving
//
#if 0
std::vector<Edge> simplifyEdges(const std::vector<Edge> edges)
{
    std::vector<Edge> result;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> shortestEdgeMap;

    // Build a map of (from,to) => (index,cost)
    // For multiple (from,to) only the smallest cost is stored
    for (int idx = 0; idx < static_cast<int>(edges.size()); ++idx) {
        const auto& edge = edges[idx];
        auto fromto = std::make_pair(edge.from, edge.to);
        int cost = static_cast<int>(edge.path.size());

        auto it = shortestEdgeMap.find(fromto);
        if (it == shortestEdgeMap.end() || cost < it->second.second) {
            shortestEdgeMap[fromto] = { idx, cost };
        }
    }
    // Get each Index from the map and store the Edge for it
    for (const auto& entry : shortestEdgeMap) {
        result.push_back(edges[entry.second.first]);
    }

    return result;
}
#endif

// The main function that computes the optimized set of AbstractEdges (MST) connecting all AbstractNodes.
std::vector<AbstractEdge> AbstractMST::generateMSTAbstractEdges(const BaseGraph& graph,
	const std::vector<Edge>& edges,
	const std::vector<Point>& baseNodes,
	const std::vector<AbstractNode>& abstractNodes)
{
    std::vector<AbstractEdge> candidates;
	std::cerr << "##MST: generateMSTAbstractEdges: for " << abstractNodes.size() << " abstract nodes" << std::endl;
    // std::vector<Edge> simpleEdges = simplifyEdges(edges);
    computeCandidateEdges(graph, edges, baseNodes, abstractNodes, candidates);
    return buildMST(candidates, static_cast<int>(abstractNodes.size()));
}
