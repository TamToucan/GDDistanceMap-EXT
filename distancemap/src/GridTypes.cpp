#include "GridTypes.hpp"
#include <queue>
#include <iostream>

namespace GridType
{

std::vector<int> checkConnectivity(const BaseGraph& graph, int numBaseNodes)
{
	std::vector<int> result;
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
		return result;
	}

	q.push(startNode);
	visited[startNode] = true;
	int reachableCount = 0;

	while (!q.empty()) {
		int node = q.front();
		q.pop();
		reachableCount++;

		for (const BaseGraphInfo& adj : graph[node]) {
			if (!visited[adj.neighbor]) {
				visited[adj.neighbor] = true;
				q.push(adj.neighbor);
			}
		}
	}

	if (reachableCount == numBaseNodes) {
		return result;
	}
	for (int i = 0; i < numBaseNodes; i++) {
		if (!visited[i]) {
			result.push_back(i);
		}
	}
	return result;
}

//
// Make a graph edgeNode1 -> list of (edgeNode2, edgeIndex, 1->2 flag, cost)
// So 1->2 flag is true if edge is Node1 -> Node2
//
BaseGraph buildBaseGraph(const std::vector<Edge>& edges, int numBaseNodes)
{
	std::cerr << "## BuildBaseGraph: " << numBaseNodes << std::endl;
	BaseGraph graph;
	graph.assign(numBaseNodes, std::vector<BaseGraphInfo>());
	for (int i = 0; i < static_cast<int>(edges.size()); i++) {
		const Edge& e = edges[i];
		if (e.toDeadEnd)
			continue; // Skip edges that lead to dead ends.
		int cost = static_cast<int>(e.path.size());
		// Add edge in forward direction.
		graph[e.from].push_back({ e.to, i, true, cost });
		// Add edge in reverse direction.
		graph[e.to].push_back({ e.from, i, false, cost });
	}
	return graph;
}

}
