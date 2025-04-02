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

// This helper function builds the base graph's adjacency list from the given edges,
// skipping any edge where toDeadEnd is true.
struct AdjEntry {
    int neighbor;   // Neighboring base node index.
    int edgeIndex;  // Index into the edges vector.
    bool forward;   // True if traversing in the stored direction.
    int cost;       // Cost is the length of the edge path.
};

void checkConnectivity(int numBaseNodes, const std::vector<std::vector<AdjEntry>>& graph) {
	std::vector<bool> visited(numBaseNodes, false);
	std::queue<int> q;

	int startNode = -1;
	for (int i = 0; i < numBaseNodes; i++) {
		if (!graph[i].empty()) {
			startNode = i;  // Pick a node with edges to start
			break;
		}
	}
	if (startNode == -1) {
		std::cerr << "Error: No base nodes with edges found!\n";
		return;
	}

	q.push(startNode);
	visited[startNode] = true;
	int reachableCount = 0;

	while (!q.empty()) {
		int node = q.front();
		q.pop();
		reachableCount++;

		for (const AdjEntry& adj : graph[node]) {
			if (!visited[adj.neighbor]) {
				visited[adj.neighbor] = true;
				q.push(adj.neighbor);
			}
		}
	}

	if (reachableCount == numBaseNodes) {
		std::cerr << "XOXOXO: All base nodes are connected.\n";
	}
	else {
		std::cerr << "XOXOXO: ERROR: Only " << reachableCount
			<< " out of " << numBaseNodes
			<< " base nodes are reachable!\n";
		for (int i = 0; i < numBaseNodes; i++) {
			if (!visited[i]) {
				std::cerr << "XOXOXO: Unreachable node: " << i << std::endl;
			}
		}
	}
}


void buildBaseGraph(const std::vector<Edge>& edges, int numBaseNodes,
                    std::vector<std::vector<AdjEntry>>& graph)
{
	std::cerr << "MST: buildBaseGraph: " << numBaseNodes << std::endl; 
    graph.assign(numBaseNodes, std::vector<AdjEntry>());
    for (int i = 0; i < static_cast<int>(edges.size()); i++) {
        const Edge& e = edges[i];
        if (e.toDeadEnd)
            continue; // Skip edges that lead to dead ends.
        int cost = static_cast<int>(e.path.size());
        // Add edge in forward direction.
        graph[e.from].push_back({e.to, i, true, cost});
        // Add edge in reverse direction.
        graph[e.to].push_back({e.from, i, false, cost});
		std::cerr << "  EDGE FROM: " << e.from << "<->" << e.to << " cost:" << cost << std::endl;
    }
	checkConnectivity(numBaseNodes, graph);
}

// Dijkstra's algorithm: given a starting base node, compute the shortest path to all base nodes.
// Returns the distance vector, predecessor vector, and usedEdge vector (which stores the edge index and direction used).
void runDijkstra(int start, const std::vector<std::vector<AdjEntry>>& graph,
                 std::vector<int>& dist, std::vector<int>& prev,
                 std::vector<std::pair<int, bool>>& usedEdge)
{
    int numNodes = static_cast<int>(graph.size());
    dist.assign(numNodes, std::numeric_limits<int>::max());
    prev.assign(numNodes, -1);
    usedEdge.assign(numNodes, std::make_pair(-1, true));

    using NodeCost = std::pair<int, int>;  // (cost, base node)
    std::priority_queue<NodeCost, std::vector<NodeCost>, std::greater<NodeCost>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
		std::cerr << "    DIJ: " << d << " dst[" << u <<"]:" << dist[u] << std::endl;
        if (d > dist[u])
            continue;
        for (const AdjEntry &adj : graph[u]) {
            int v = adj.neighbor;
            int nd = d + adj.cost;
			std::cerr << "       CHK: neigh:" << v << " edg:" << adj.edgeIndex << " cost:" << adj.cost << " newDist: " << nd << std::endl;
            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                usedEdge[v] = { adj.edgeIndex, adj.forward };
				std::cerr << "        ADD " << nd << " " << v << std::endl;
                pq.push({ nd, v });
            }
            else
            {
				std::cerr << "        nd >= dist[v] " << nd << " >= " << dist[v] << std::endl;
            }
        }
    }
}

// Reconstruct the path (as a sequence of Points) between two base nodes given the Dijkstra predecessors.
Path reconstructPath(int start, int target, const std::vector<int>& prev,
                     const std::vector<std::pair<int, bool>>& usedEdge,
                     const std::vector<Edge>& edges)
{
    std::vector<std::pair<int, bool>> edgeSequence;
    int cur = target;
    while (cur != start) {
        edgeSequence.push_back(usedEdge[cur]);
        cur = prev[cur];
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
void computeCandidateEdges(const std::vector<Edge>& edges,
                           const std::vector<Point>& baseNodes,
                           const std::vector<AbstractNode>& abstractNodes,
                           std::vector<AbstractEdge>& candidates)
{
    int numBaseNodes = static_cast<int>(baseNodes.size());
    std::vector<std::vector<AdjEntry>> baseGraph;
    buildBaseGraph(edges, numBaseNodes, baseGraph);

    int n = static_cast<int>(abstractNodes.size());
    candidates.clear();

    // For each abstract node as a source:
	std::cerr << "MST: computeCandidateEdges: " << n << std::endl;
    for (int i = 0; i < n; i++) {
        int startBase = abstractNodes[i].baseCenterNode;
        std::vector<int> dist;
        std::vector<int> prev;
        std::vector<std::pair<int, bool>> usedEdge;
		std::cerr << "  Dijkstra for baseNode: " << startBase << std::endl;
        runDijkstra(startBase, baseGraph, dist, prev, usedEdge);

        std::cerr << "Distances from baseNode " << startBase << ": ";
        for (int d : dist) {
            if (d == std::numeric_limits<int>::max()) std::cerr << "**************** INF **************" << std::endl;
        }
        std::cerr << std::endl;

        // For every other abstract node j > i, record the candidate edge.
		std::cerr << "  MST loop: " << i+1 << " << to " << n-1 << std::endl;
        for (int j = i + 1; j < n; j++) {
            int targetBase = abstractNodes[j].baseCenterNode;
			std::cerr << "       check dist of " << targetBase << " base" << std::endl;   
            if (dist[targetBase] == std::numeric_limits<int>::max())
                continue; // Not reachable.
            AbstractEdge ce;
            ce.from = i;
            ce.to = j;
            ce.path = reconstructPath(startBase, targetBase, prev, usedEdge, edges);
            candidates.push_back(ce);
			std::cerr << "       MAKE CANDIDATE: " << i << "->" << j << " path: " << ce.path.size() << std::endl;
			for (const auto& p : ce.path) {
                std::cerr << "         " << p.first << "," << p.second << "  ";
			}
        	std::cerr << std::endl;
        }
    }
}

// Run Kruskal's algorithm on the candidate edges to build the MST over abstract nodes.
void buildMST(const std::vector<AbstractEdge>& candidates, int numAbstractNodes,
              std::vector<AbstractEdge>& mstEdges)
{
    // Start with all candidate edges and sort them by weight.
    std::vector<AbstractEdge> sortedEdges = candidates;
    std::sort(sortedEdges.begin(), sortedEdges.end(),
              [](const AbstractEdge &a, const AbstractEdge &b) {
                  return a.path.size() < b.path.size();
              });

    // Use a union-find structure to detect cycles.
    UnionFind uf(numAbstractNodes);
    mstEdges.clear();

	std::cerr << "MST Build: edges: " << sortedEdges.size() << std::endl;
    for (const AbstractEdge& ce : sortedEdges) {
        int setA = uf.find(ce.from);
        int setB = uf.find(ce.to);
		std::cerr << "  setA: " << setA << " setB: " << setB << std::endl;
        if (setA != setB) {
            uf.unite(setA, setB);
            mstEdges.push_back(ce);
			std::cerr << "  ADD MST EDGE: " << ce.from << "->" << ce.to << std::endl;
            // Stop if we have n-1 edges.
            if (mstEdges.size() == static_cast<size_t>(numAbstractNodes - 1))
                break;
        }
    }
}

// The main function that computes the optimized set of AbstractEdges (MST) connecting all AbstractNodes.
void AbstractMST::generateMSTAbstractEdges(const std::vector<Edge>& edges,
                              const std::vector<Point>& baseNodes,
                              const std::vector<AbstractNode>& abstractNodes,
                              std::vector<AbstractEdge>& abstractEdges)
{
    std::vector<AbstractEdge> candidates;
    computeCandidateEdges(edges, baseNodes, abstractNodes, candidates);
    buildMST(candidates, static_cast<int>(abstractNodes.size()), abstractEdges);
}
