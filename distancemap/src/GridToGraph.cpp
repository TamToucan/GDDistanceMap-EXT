#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <utility>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <set>

#include "GridToGraph.hpp"
#include "ZSThinning.hpp"

#include "LineMover.h"
#include "TGA.hpp"

namespace {

// Helper function to check if a point is within bounds
inline
bool isInBounds(int x, int y, int rows, int cols) {
    return x >= 0 && y >= 0 && x < cols && y < rows;
}


using Pattern = std::vector<std::vector<int>>;

// Define all patterns for nodes(middle point is the node)
//
const std::vector<Pattern>& getBasePatterns() {
    static std::vector<Pattern> patterns;

    if (patterns.size() != 0) {
    	return patterns;

    }
    // 4-Way Junction (+-Junction)
    patterns.push_back({
        {0, 1, 0 },
        {1, 1, 1 },
        {0, 1, 0 },
    });

    // 4-Way Junction (X-Junction)
    patterns.push_back({
        {1, 0, 1 },
        {0, 1, 0 },
        {1, 0, 1 },
    });
    // 1-Way Junction (X-Junction)
    patterns.push_back({
        {1, 0, 1 },
        {0, 1, 0 },
        {1, 0, 0 },
    });
    // 1-Way Junction (X-Junction) 1 filled
    patterns.push_back({
        {1, 0, 1 },
        {1, 1, 0 },
        {1, 0, 0 },
    });
    // T-Junction
    patterns.push_back({
        {0, 1, 0 },
        {1, 1, 1 },
        {0, 0, 0 },
    });
    // T-junc with 1 diag
    patterns.push_back({
        {0, 1, 0 },
        {0, 1, 1 },
        {1, 0, 0 },
    });

    // 3-way with overlap (1 unset)
    patterns.push_back({
        {0, 0, 1 },
        {1, 1, 1 },
        {0, 1, 0 },
    });
    // 3-way with overlap
    patterns.push_back({
        {0, 1, 1 },
        {1, 1, 1 },
        {0, 0, 1 },
    });

    // T-jun 2 diag
    patterns.push_back({
        {0, 1, 0 },
        {0, 1, 0 },
        {1, 0, 1 },
    });

    // T-jun 2 diag (top set)
    patterns.push_back({
        {0, 1, 1 },
        {0, 1, 0 },
        {1, 0, 1 },
    });

    // T-jun 2 diag (mid set)
    patterns.push_back({
        {0, 1, 0 },
        {0, 1, 1 },
        {1, 0, 1 },
    });

    // 2 diag unset
    patterns.push_back({
        {1, 0, 1 },
        {0, 1, 1 },
        {1, 1, 1 },
    });

    // 2 adjacent unset
    patterns.push_back({
        {1, 1, 1 },
        {0, 1, 1 },
        {0, 1, 1 },
    });

    // 1 unset
    patterns.push_back({
        {1, 1, 1 },
        {1, 1, 0 },
        {1, 1, 1 },
    });

    // T-junc with (bottom set)
    patterns.push_back({
        {0, 1, 0 },
        {1, 1, 1 },
        {1, 0, 0 },
    });

    // T-junc with (both bottom set)
    patterns.push_back({
        {0, 1, 0 },
        {1, 1, 1 },
        {1, 0, 1 },
    });
    return patterns;
}

// Mirror the 3x3 pattern
//
Pattern mirrorPattern(const Pattern& pattern) {
    Pattern mirrored(3, std::vector<int>(3, 0));
    for (int i = 0; i < 3; ++i) {
    	mirrored[i][0] = pattern[i][2];
    	mirrored[i][1] = pattern[i][1];
    	mirrored[i][2] = pattern[i][0];
    }
    return mirrored;
}

// Rotate the 3x3 pattern
//
Pattern rotatePattern(const Pattern& pattern) {
    Pattern rotated(3, std::vector<int>(3, 0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotated[j][(3-1) - i] = pattern[i][j]; // Rotate clockwise
        }
    }
    return rotated;
}

// Use the 9 (3x3) bits (bit0 = top left) to created pattern ID
//
int getPatternID(const Pattern& grid, int x, int y) {
	int id = 0;
	int bit = 0x01;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (grid[y+i][x+j] == GridToGraph::PATH) {
				id |= bit;
			}
			bit <<= 1;
    	}
	}
	return id;
}

// Add the pattern to the list of IDs if not already present
// (some patterns are symmetrical so rotated and/or mirrored
// wont need added since they generate same ID since same pattern
//
bool addPatternID(const Pattern& p, std::vector<int>& ids) {
	int id = getPatternID(p, 0, 0);
	// Return true if first time ever stored id
	bool ret = (ids[id] == 0);
	ids[id] = 1;
	return ret;
}

//
// Take the base patterns and rotation them 0,90,180,270 degrees
// also mirror then rotate
// The returned array contains all bit pattners (using the 9 (3x3) bits)
// of patterns that the middle of 3x3 is a node
//
const std::vector<int>& getAllPatterns() {
    static std::vector<Pattern> allPatterns;
    static std::vector<int> patternIDs(1 << 3*3, 0);
    static bool madePatterns = false;

    if (!madePatterns) {
    	for (bool mirror : { false, true }) {
    		for (const auto& pattern : getBasePatterns()) {
    			Pattern current = mirror ? mirrorPattern(pattern) : pattern;
    			for (int r = 0; r < 3; ++r) {
    				if (addPatternID(current, patternIDs)) {
    					allPatterns.push_back(current);
    				}
    				current = rotatePattern(current);
    			}
    			if (addPatternID(current, patternIDs)) {
    				allPatterns.push_back(current);
    			}
    		}
    	}
    	madePatterns = true;
    }

    return patternIDs;
}

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};


// The is only used by Deadend detection and only will only return 1 if
// all EMPTY and 1 PATH since PAHT==1 and EMPTY==0 and all others > 1.
//
inline
int countNeighbors(const GridToGraph::Grid& grid, int x, int y) {
	return grid[y-1][x-1]
		+ grid[y-1][x]
		+ grid[y-1][x+1]

		+ grid[y][x-1]
		+ grid[y][x+1]

		+ grid[y+1][x-1]
		+ grid[y+1][x]
		+ grid[y+1][x+1];
}
}

namespace GridToGraph {

///////////////////////////////////////////////////////////

std::vector<Point> detectDeadEnds(const Grid& grid)
{
    std::vector<std::pair<int, int>> deadEnds;
    for (int y = 1; y < grid.size() - 1; ++y) {
        for (int x = 1; x < grid[0].size() - 1; ++x) {
        	// NOTE: This relies on PATH==1
        	if (grid[y][x] && countNeighbors(grid, x,y) == 1) {
        		deadEnds.push_back({x, y});
        	}
        }
    }

    return deadEnds;
}

std::vector<Point> detectNodes(const Grid& grid)
{
    std::vector<std::pair<int, int>> nodes;
    const auto& patternIDS = getAllPatterns();
    for (int y = 0; y < grid.size() - 2; ++y) {
        for (int x = 0; x < grid[0].size() - 2; ++x) {
        	// Check that the middle of the 3x3 to be checked is set
        	// (could skip this since all patterns have middle bit set)
        	//
            if (grid[y+1][x+1]) {
            	// Get the ID from the 9 (3x3) bits and check if a pattern exists
            	int id = getPatternID(grid, x, y);
            	if (patternIDS[id]) {
            		// Pattern exists (so it was a match) => store the Node
                	nodes.push_back({x+1, y+1});
            	}
            }
        }
    }

    return nodes;
}

///////////////////////////////////////////////////////////

namespace {
	void makeTGA(const char* name, const Grid& grid, unsigned char forZero=0x00)
	{
		unsigned char* pPixel = new unsigned char[grid.size()*grid[0].size()*4];
		unsigned char* pData = pPixel;
		for (int y=grid.size()-1; y>= 0; --y) {
			for (int xy : grid[y]) {
				switch (xy)
				{
				case GridToGraph::NODE:
					*pData++ = 0xff;
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				case GridToGraph::XPND:
					*pData++ = 0x7f;
					*pData++ = 0x7f;
					*pData++ = 0x00;
					*pData++ = 0x7f;
					break;
				case GridToGraph::DEND:
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				case 0x05:
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0xff;
					*pData++ = 0xff;
					break;
				case 0xff:
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0x00;
					break;
				default:
					*pData++ = xy ? ~forZero : forZero;
					*pData++ = xy ? ~forZero : forZero;
					*pData++ = xy ? ~forZero : forZero;
					*pData++ = xy ? ~forZero : forZero;
					break;
				}
			}
		}
		Stuff::TGA::saveToFile(name, grid[0].size(), grid.size(), 32, pPixel);
		delete[] pPixel;
	}
}

///////////////////////////////////////////////////////////

    // Directions for 8 neighbouring cells
std::vector<Point> directions = {
		{-1, 0}, {1, 0}, {0, -1}, {0, 1}, // Up, Down, Left, Right
		{-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Diagonals
};

// Helper function to check if a point is valid in the grid
bool isValid(int x, int y, const Grid& grid, const std::vector<std::vector<bool>>& visited) {
    return isInBounds(x,y, grid.size(), grid[0].size()) && grid[y][x] == GridToGraph::PATH && !visited[y][x];
}

//////////////////////////////////////////////////////////////////////////

// Trace a single path from the start point until another node or dead end is found
Path tracePath(Point start, const Grid& grid, std::vector<std::vector<bool>>& visited) {
    Path path;
    Point current = start;
    path.push_back(current);
    visited[current.second][current.first] = true;

    while (true) {
        bool foundNext = false;

        // Explore all directions
        for (const auto& dir : directions) {
            int nx = current.first + dir.first;
            int ny = current.second + dir.second;

            if (isValid(nx, ny, grid, visited)) {
                current = {nx, ny};
                path.push_back(current);
                visited[ny][nx] = true;

                // If we reach another node or dead end, stop the traversal
                if (grid[ny][nx] == GridToGraph::NODE || grid[ny][nx] == GridToGraph::DEND) {
                    return path;
                }

                foundNext = true;
                break;
            }
        }

        // If no further valid points are found, end the traversal
        if (!foundNext) {
            break;
        }
    }

    return path;
}

/////////////////////////////////////////////////////////////////////////////////////////

//
// Set the Node and DeadEnds
//
void updateGrid(Grid& grid, const std::vector<Point>& nodes, const std::vector<Point>& deadEnds) {
    for (const auto& n : nodes) {
    	grid[n.second][n.first] = GridToGraph::NODE;
    }
    for (const auto& de : deadEnds) {
    	grid[de.second][de.first] = GridToGraph::DEND;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Method to find edges
std::vector<Edge> findEdges(const Grid& grid, const std::vector<Point>& points, const std::vector<Point>& deadEnds) {
    // Combine nodes and deadEnds for easier lookup
    std::vector<Point> allPoints = points;
    allPoints.insert(allPoints.end(), deadEnds.begin(), deadEnds.end());

    // Set up the visited grid for efficiency
    Grid visited(grid.size(), std::vector<int>(grid[0].size(), 0));

    std::vector<Edge> edges;

    // Process each node and deadEnd
    for (size_t i = 0; i < allPoints.size(); ++i) {
        const auto& start = allPoints[i];

        // Skip already visited points
        if (visited[start.second][start.first]) {
            continue;
        }

        // BFS to explore paths
        std::queue<std::pair<Point, std::vector<Point>>> q;
        q.push({start, {start}}); // Initialize with the current point

        while (!q.empty()) {
            auto [current, path] = q.front();
            q.pop();

            int x = current.first;
            int y = current.second;

            // Mark current point as visited
            visited[y][x] = 1;

            // Explore neighbors
            for (const auto& [dx, dy] : directions) {
                int nx = x + dx;
                int ny = y + dy;

                // Skip invalid or already visited points
                if (!isInBounds(nx, ny, grid.size(), grid[0].size()) || visited[ny][nx] || grid[ny][nx] == GridToGraph::EMPTY) {
                    continue;
                }

                Point neighbor = {nx, ny};
                path.push_back(neighbor);

                // Check if the neighbor is a node or deadEnd
                auto it = std::find(allPoints.begin(), allPoints.end(), neighbor);
                if (it != allPoints.end()) {
                    int toIndex = (it - allPoints.begin());
                    bool isDeadEnd = (toIndex >= points.size());
                    toIndex -= (isDeadEnd ? points.size() : 0);

                    // Add an edge
                    edges.push_back({static_cast<int>(i), toIndex, isDeadEnd, path});

                    // Stop further exploration along this path
                    break;
                }

                // Continue BFS if the neighbor is part of the path
                q.push({neighbor, path});
            }
        }
    }

    return edges;
}

/////////////////////////////////////////////////////////////////////////////////////////

#if 0
// Try to expand the outline for a single node
bool tryExpandNode(Point node, Grid& grid, int expansionSize)
{
    const int rows = grid.size();
    const int cols = grid[0].size();

    // Count the number of valid points in the new layer
    int layerStart = -expansionSize;
    int layerEnd = expansionSize;
    std::vector<Point> expansionPoints;

    for (int dx = layerStart; dx <= layerEnd; ++dx) {
        for (int dy = layerStart; dy <= layerEnd; ++dy) {
            // Check only the boundary of the current square
            if (abs(dx) == expansionSize || abs(dy) == expansionSize) {
                int nx = node.first + dx;
                int ny = node.second + dy;

                if (isInBounds(nx, ny, cols, rows) && (grid[ny][nx] == GridToGraph::EMPTY || grid[ny][nx] == GridToGraph::PATH)) {
                    expansionPoints.push_back({nx, ny});
                }
            }
        }
    }

    // If more than half the outline is not empty or PATH then we done expand
    int totalPoints = (2 * expansionSize + 1) * (2 * expansionSize + 1)
    		        - (2 * (expansionSize - 1) + 1) * (2 * (expansionSize - 1) + 1);
    if (totalPoints / 2 >= expansionPoints.size()) {
    	return false;
    }

    // Expand the outline
    for (const auto& p : expansionPoints) {
    	grid[p.second][p.first] = GridToGraph::XPND;
    }
    return true;
}

void expandNodes(Grid& grid, const std::vector<Point>& nodes)
{
    std::unordered_set<Point, PairHash> completedNodes(0);

    // Continue expanding until no node can expand further
    int expansionSize = 1;
    bool expanded = true;
    while (expanded) {
        expanded = false;

        for (const auto& node : nodes) {
            if (completedNodes.find(node) != completedNodes.end()) {
                continue;
            }

            if (tryExpandNode(node, grid, expansionSize)) {
                expanded = true;
            } else {
                completedNodes.insert(node); // Mark this node as completed
            }
        }

        ++expansionSize;
    }
}
#endif

//////////////////////////////////////////////////////////////////////

void thickenPaths(Grid& grid)
{
    int rows = grid.size();
    int cols = grid[0].size();

    // Queue to keep track of points to process
    std::queue<Point> queue;

    // Initialize the queue with all path points
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c] == GridToGraph::PATH) {
                queue.push({r, c});
            }
        }
    }

    // Process points in the queue
    while (!queue.empty()) {
        auto [r, c] = queue.front();
        queue.pop();

        // Attempt to thicken in all 8 directions
        for (const auto& [dr, dc] : directions) {
            int nr = r + dr;
            int nc = c + dc;

            // Check if the neighboring cell is within bounds and is empty
            if (isInBounds(nc, nr, rows, cols) && grid[nr][nc] == GridToGraph::EMPTY) {
                grid[nr][nc] = GridToGraph::XPND;
                queue.push({nr, nc});
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////

// Helper function to calculate Euclidean distance
double euclideanDistance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.first - b.first, 2) + std::pow(a.second - b.second, 2));
}


// Find the closest node to the centroid of an AbstractNode
int findCentralNode(const AbstractNode& abstractNode, const std::vector<Point>& nodes) {
    if (abstractNode.baseNodes.empty()) return -1;

    Point centroid = abstractNode.center;
    int closestNode = abstractNode.baseNodes[0];
    double minDistance = euclideanDistance(centroid, nodes[closestNode]);
    for (int nodeIdx : abstractNode.baseNodes) {
        double d = euclideanDistance(centroid, nodes[nodeIdx]);
        if (d < minDistance) {
            minDistance = d;
            closestNode = nodeIdx;
        }
    }
    return closestNode;
}

// Find the path between two nodes in the base graph
Path findPathBetweenNodes(int startNode, int endNode, const std::vector<Edge>& edges, const std::vector<Point>& nodes)
{
    // BFS for simplicity (A* could be used for better performance)
    std::queue<int> q;
    std::vector<int> prev(nodes.size(), -1); // For reconstructing the path
    std::vector<bool> visited(nodes.size(), false);

    q.push(startNode);
    visited[startNode] = true;

    while (!q.empty()) {
        int current = q.front();
        q.pop();

        if (current == endNode) break;

        for (const auto& edge : edges) {
            int neighbor = -1;
            if (edge.from == current) neighbor = edge.to;
            else if (edge.to == current) neighbor = edge.from;

            if (neighbor != -1 && !visited[neighbor]) {
                visited[neighbor] = true;
                prev[neighbor] = current;
                q.push(neighbor);
            }
        }
    }

    // Reconstruct the path
    std::vector<Point> path;
    for (int at = endNode; at != -1; at = prev[at]) {
        path.push_back(nodes[at]);
    }
    std::reverse(path.begin(), path.end());
    return path;
}



//////////////////////////////////////////////////////////////////////

std::vector<int> dbscan(const std::vector<Point>& points, double eps, int minPts)
{
    const int UNVISITED = -1;
    const int NOISE = -2;

    std::vector<int> labels(points.size(), UNVISITED);
    int clusterId = 0;

    auto regionQuery = [&](const Point& p) -> std::vector<int> {
        std::vector<int> neighbors;
        for (size_t i = 0; i < points.size(); ++i) {
            double d = euclideanDistance(p, points[i]);
            //if (euclideanDistance(p, points[i]) <= eps) {
            if (d <= eps) {
            	//std::cerr << "POINT: " << p.first << "," << p.second << " to " << points[i].first << "," << points[i].second << " dist: " << d << std::endl;
                neighbors.push_back(i);
            }
        }
        return neighbors;
    };

    auto expandCluster = [&](int pointIdx, int clusterId, const std::vector<int>& neighbors) {
        labels[pointIdx] = clusterId;

        std::vector<int> queue = neighbors;
        while (!queue.empty()) {
            int idx = queue.back();
            queue.pop_back();

            if (labels[idx] == UNVISITED) {
                labels[idx] = clusterId;

                auto subNeighbors = regionQuery(points[idx]);
                if (subNeighbors.size() >= static_cast<size_t>(minPts)) {
                	std::cerr << "PIDX: " << pointIdx << " id: " << clusterId << " neigh: " << subNeighbors.size() << std::endl;
                    queue.insert(queue.end(), subNeighbors.begin(), subNeighbors.end());
                }
            }

            if (labels[idx] == NOISE) {
                labels[idx] = clusterId;
            }
        }
    };

    for (size_t i = 0; i < points.size(); ++i) {
        if (labels[i] != UNVISITED) {
            continue;
        }

        auto neighbors = regionQuery(points[i]);
        if (neighbors.size() < static_cast<size_t>(minPts)) {
            labels[i] = NOISE;
        } else {
        	std::cerr << "## expand: " << i << " clust: " << clusterId << " neigh: " << neighbors.size() << std::endl;
            expandCluster(i, clusterId, neighbors);
            ++clusterId;
        }
    }

    return labels;
}

//////////////////////////////////////////////

AbstractGraph createAbstractGraph(
    const std::vector<Point>& nodes,
    const std::vector<Point>& deadEnds,
    const std::vector<Edge>& edges,
    double clusteringEps,
    int minClusterSize)
{
    // Combine nodes and deadEnds into one set for clustering
    std::vector<Point> allPoints = nodes;
    allPoints.insert(allPoints.end(), deadEnds.begin(), deadEnds.end());

    // Step 1: Cluster the combined points
    std::vector<int> clusterLabels = dbscan(allPoints, clusteringEps, minClusterSize);
    std::cerr << "LABELS: " << clusterLabels.size() << " from nodes+dead:" << allPoints.size() << std::endl;
	for (int i=0; i < clusterLabels.size(); ++i) {
			std::cerr << "   [" << i << "] = " << clusterLabels[i] << std::endl;
	}
    // Create abstract nodes from clusters
    std::unordered_map<int, AbstractNode> clusters;
    for (size_t i = 0; i < allPoints.size(); ++i) {
        int clusterId = clusterLabels[i];
        if (clusterId < 0) {
            continue; // Skip noise points
        }

        if (clusters.find(clusterId) == clusters.end()) {
            clusters[clusterId] = {clusterId, {}, {}, {0, 0}};
        }

        if (i < nodes.size()) {
            clusters[clusterId].baseNodes.push_back(static_cast<int>(i));
            std::cerr << "ABNOD " << clusters.size() << ": " << clusterId << " + " << i << std::endl;
        } else {
            clusters[clusterId].baseDeadEnds.push_back(static_cast<int>(i - nodes.size()));
            std::cerr << "ABDED " << clusters.size() << ": " << clusterId << " + " << (i-nodes.size()) << std::endl;
        }

        clusters[clusterId].center.first += allPoints[i].first;
        clusters[clusterId].center.second += allPoints[i].second;
    }

    // Finalize cluster centers
    for (auto& [_, cluster] : clusters) {
        int totalPoints = cluster.baseNodes.size() + cluster.baseDeadEnds.size();
        cluster.center.first /= totalPoints;
        cluster.center.second /= totalPoints;
    }

    // Step 2: Create abstract edges
    std::cerr <<"===CREATE AB EDGES input: " << edges.size() << std::endl;
    std::vector<AbstractEdge> abstractEdges;
    for (int edgeIdx=0; edgeIdx < edges.size(); ++edgeIdx) {
    	const auto& edge = edges[edgeIdx];
    	std::cerr << "CHECK EDGE: " << edge.from << " to " << edge.to << " c:" << edge.path.size() << " isDead:" << edge.toDeadEnd << std::endl;
        int clusterFrom = clusterLabels[edge.from];
        int clusterTo = edge.toDeadEnd ? clusterLabels[nodes.size() + edge.to]
                                       : clusterLabels[edge.to];

        if (clusterFrom < 0 || clusterTo < 0 || clusterFrom == clusterTo) {
        	std::cerr << "              FROM: " << clusterFrom << " to: " << clusterTo << " SKIP" << std::endl;
            continue; // Skip edges involving noise or intra-cluster edges
        }

        double cost = edge.path.size();
    	std::cerr << "ABEDG: " << (edge.toDeadEnd ? "DEND " : "EDGE ") << " from:" << edge.from << " to:"
    			<< edge.to << " +" << (edge.toDeadEnd ? nodes.size() : 0) << ") "
    			<< " gives clust from: " << clusterFrom << " to: " << clusterTo
				<< " cost: " << cost << std::endl;
        abstractEdges.push_back({clusterFrom, clusterTo });
    }

    // Step 3: Collect abstract nodes into a vector
    std::vector<AbstractNode> abstractNodes;
    for (const auto& [_, cluster] : clusters) {
        abstractNodes.push_back(cluster);
    }

    // Step 4: Find Abstract Edge path
    std::unordered_map<std::pair<int,int>, Path, PairHash> fromtoPathMap;
    for (auto& abEdge : abstractEdges) {
    	// Find central nodes for `from` and `to`
        int fromCentral = findCentralNode(abstractNodes[abEdge.from], nodes);
        int toCentral = findCentralNode(abstractNodes[abEdge.to], nodes);

        auto fp = fromtoPathMap.find({fromCentral, toCentral});
        if (fp != fromtoPathMap.end()) {
        	abEdge.path = fp->second;
        }
        else {
        	// Find the path between the central nodes
        	abEdge.path = findPathBetweenNodes(fromCentral, toCentral, edges, nodes);
        	fromtoPathMap[{fromCentral, toCentral}] = abEdge.path;
        	std::cerr << "::::: " << fromCentral <<" " << toCentral << " " << abEdge.path.size() << std::endl;
        }
    }

    return {abstractNodes, abstractEdges};
}

//////////////////////////////////////////////////////////////////////

void makeGraph(const Grid& floorGrid)
{
	makeTGA("GRID.tga", floorGrid, 0xff);
	// Need to copy the floorGrid since ZSThinning removes floor
	Grid thinGrid(floorGrid);
    Algo::ZSThinning(thinGrid);

	makeTGA("THIN.tga", thinGrid);
	std::vector<Point> deadends = detectDeadEnds(thinGrid);
	std::vector<Point> nodes = detectNodes(thinGrid);
	updateGrid(thinGrid, nodes, deadends);
	std::vector<Edge> edges = findEdges(thinGrid, nodes, deadends);
	makeTGA("GRAPH.tga", thinGrid);


	{
		Grid tempGrid = floorGrid;
		for (const Edge& e : edges) {
			const Point& f = nodes[e.from];
			const Point& t = (e.toDeadEnd ? deadends[e.to] : nodes[e.from]);
			for (const auto& p : e.path) {
				tempGrid[p.second][p.first] = 0x05;
			}
			tempGrid[f.second][f.first] = GridToGraph::NODE;
			tempGrid[t.second][t.first] = e.toDeadEnd ? GridToGraph::DEND : GridToGraph::NODE;
		}
		makeTGA("EDGES.tga", tempGrid, 0xff);
	}

	// Before can expand need to put back the WALLs
	// i.e. floorGrid had zeroes wherever walls, but thinGrid
	// has zeroes everywhere except path. So change thinGrid
	// to have WALL wherever was 0 in floorGrid
    int rows = floorGrid.size();
    int cols = floorGrid[0].size();
	for (int y=0; y < rows; ++y) {
		for (int x=0; x < cols; ++x) {
			if (floorGrid[y][x] != GridToGraph::PATH) {
				thinGrid[y][x] = 0xff;
			}
		}
	}
	thickenPaths(thinGrid);

	//expandNodes(thinGrid, nodes);
	makeTGA("XPND.tga", thinGrid);

	auto [abstractNodes, abstractEdges] = createAbstractGraph(nodes, deadends, edges, 5.0, 2);
	{
		std::cerr << "\nAbstract Nodes:  " << abstractNodes.size() << std::endl;
		for (const auto& node : abstractNodes) {
			std::cerr << "Cluster " << node.id << " Center: (" << node.center.first << ", " << node.center.second << ")"
					<< " size dend:" << node.baseDeadEnds.size() << " node: " << node.baseNodes.size() << std::endl;
		}

		std::cerr << "\nAbstract Edges: " << abstractEdges.size() << std::endl;
		for (const auto& edge : abstractEdges) {
			std::cerr << "Edge from Cluster " << edge.from << " to Cluster " << edge.to << " with cost " << edge.path.size() << std::endl;
		}
	}
	for (const auto& e : abstractEdges) {
		std::cerr << "EDGE: " << e.from << " to " << e.to << std::endl;
		const auto& f = abstractNodes[e.from];
		const auto& t = abstractNodes[e.to];
		std::cerr << "POINT: " << f.center.first << "," << f.center.second << " to: " << t.center.first << "," << t.center.second << std::endl;
		MathStuff::LineMover lm(f.center.first, f.center.second, t.center.first, t.center.second);
		do {
			int x = lm.getX();
			int y = lm.getY();
			std::cerr << "  LM:" << x << "," << y << std::endl;
			thinGrid[y][x] = 0x05;
		} while (lm.move(1));
	}
	makeTGA("META.tga", thinGrid);
	std::cerr << "NODES: " << nodes.size() << std::endl
			  << "DENDS: " << deadends.size() << std::endl
			  << "EDGES: " << edges.size() << std::endl
			  << std::endl
			  << "ABNOD: " << abstractNodes.size() << std::endl
			  << "ABEDG: " << abstractEdges.size() << std::endl;
}

///////////////////////////////////////////////////////////

}
