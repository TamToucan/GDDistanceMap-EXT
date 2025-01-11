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
#include "MathUtils.h"

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

void debugDump(const Graph& graph);

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
#if 0
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
					*pData++ = 0x00;
					*pData++ = 0xff;
					*pData++ = 0xff;
					*pData++ = 0xff;
					break;
				case 0x06:
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
#endif
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
void updateInfoGrid(Grid& grid, const std::vector<Point>& nodes, const std::vector<Point>& deadEnds) {
    for (const auto& n : nodes) {
    	grid[n.second][n.first] = GridToGraph::NODE;
    }
    for (const auto& de : deadEnds) {
    	grid[de.second][de.first] = GridToGraph::DEND;
    }
}

//
// Before can expand need to put back the WALLs
// i.e. floorGrid had zeroes wherever walls, but graph.infoGrid
// has zeroes everywhere except path. So change graph.infoGrid
// to have WALL wherever was 0 in floorGrid
//
void restoreWalls(Grid& infoGrid, const Grid& floorGrid) {
    int rows = floorGrid.size();
    int cols = floorGrid[0].size();
	for (int y=0; y < rows; ++y) {
		for (int x=0; x < cols; ++x) {
			if (floorGrid[y][x] != GridToGraph::PATH) {
				infoGrid[y][x] = GridToGraph::WALL;
			}
		}
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

//
// Expand all the paths to fill the EMPT spaces
//
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


////////////////////////////////////////////////////////////////////////////////////////

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


Path findPathBetweenNodes(int startNode, int endNode, const std::vector<Edge>& edges, const std::vector<Point>& nodes)
{
    // BFS for simplicity (A* could be used for better performance)
    std::queue<int> q;
    std::vector<int> prev(nodes.size(), -1);   // For reconstructing the path
    std::vector<int> edgeUsed(nodes.size(), -1); // To track the edge that was used to reach a node
    std::vector<bool> visited(nodes.size(), false);

    q.push(startNode);
    visited[startNode] = true;

    while (!q.empty()) {
        int current = q.front();
        q.pop();

        if (current == endNode) break;

        for (int edgeIdx = 0; edgeIdx < edges.size(); ++edgeIdx) {
            const auto& edge = edges[edgeIdx];
            if (edge.toDeadEnd) continue;

            int neighbor = -1;

            if (edge.from == current) neighbor = edge.to;
            else if (edge.to == current) neighbor = edge.from;

            if (neighbor != -1 && !visited[neighbor]) {
                visited[neighbor] = true;
                prev[neighbor] = current;
                edgeUsed[neighbor] = edgeIdx; // Store the edge used to reach this neighbor
                q.push(neighbor);
            }
        }
    }

    // Reconstruct the path using the edges
    Path fullPath;
    for (int at = endNode; at != -1; at = prev[at]) {
        int edgeIdx = edgeUsed[at];
        if (edgeIdx != -1) {
            const auto& edgePath = edges[edgeIdx].path;
            fullPath.insert(fullPath.end(), edgePath.rbegin(), edgePath.rend());
        }
    }

    // Reverse the path to make it go from startNode to endNode
    std::reverse(fullPath.begin(), fullPath.end());
    return fullPath;
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
            expandCluster(i, clusterId, neighbors);
            ++clusterId;
        }
    }

    return labels;
}

//////////////////////////////////////////////

//
// Cluster nodes and dead ends into AbstractNodes
// and store all the points of the base nodes/dends
// to calculate the center
// Create Abstract edges for the abstract nodes based
// on the existing base edges
// Calculate the base node closest to the center point of cluster
// Make abstract edge path from each base closted node
//
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
        } else {
            clusters[clusterId].baseDeadEnds.push_back(static_cast<int>(i - nodes.size()));
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
    std::vector<AbstractEdge> abstractEdges;
    for (int edgeIdx=0; edgeIdx < edges.size(); ++edgeIdx) {
    	const auto& edge = edges[edgeIdx];
        int clusterFrom = clusterLabels[edge.from];
        int clusterTo = edge.toDeadEnd ? clusterLabels[nodes.size() + edge.to]
                                       : clusterLabels[edge.to];

        if (clusterFrom < 0 || clusterTo < 0 || clusterFrom == clusterTo) {
            continue; // Skip edges involving noise or intra-cluster edges
        }

        double cost = edge.path.size();
        abstractEdges.push_back({clusterFrom, clusterTo });
    }

    // Step 3: Collect abstract nodes into a vector
    std::vector<AbstractNode> abstractNodes;
    for (const auto& [_, cluster] : clusters) {
    	abstractNodes.push_back(cluster);
    }

    // Keep mapping of AbstractNode idx to closest Node Idx
    std::unordered_map<int, int> closestMap;
    auto findClosest = [&](int abNodeIdx) -> int {
    	int baseIdx;
    	auto fnd = closestMap.find(abNodeIdx);
		if (fnd == closestMap.end()) {
			baseIdx = findCentralNode(abstractNodes[abNodeIdx], nodes);
			closestMap[abNodeIdx] = baseIdx;
		}
		else {
			baseIdx = fnd->second;
		}
        return baseIdx;
    };

    // Step 4: Find Abstract Edge path
    std::unordered_map<std::pair<int,int>, Path, PairHash> fromtoPathMap;
    for (auto& abEdge : abstractEdges) {
    	int fromCentral = findClosest(abEdge.from);
    	int toCentral = findClosest(abEdge.to);
    	// set base closest node
    	abstractNodes[abEdge.from].baseCenterNode = fromCentral;
    	abstractNodes[abEdge.to].baseCenterNode = toCentral;

        auto fp = fromtoPathMap.find({fromCentral, toCentral});
        if (fp != fromtoPathMap.end()) {
        	abEdge.path = fp->second;
        }
        else {
        	// Find the path between the central nodes
        	abEdge.path = findPathBetweenNodes(fromCentral, toCentral, edges, nodes);
        	fromtoPathMap[{fromCentral, toCentral}] = abEdge.path;
        }
    }

    return {abstractNodes, abstractEdges};
}

///////////////////////////////////////////////////////////////////////

// Compute direction as an index into the directions array
int computeDirection(const Point& from, const Point& to) {
    int dx = to.first - from.first;
    int dy = to.second - from.second;

    // Normalize dx and dy to -1, 0, or 1
    dx = (dx == 0) ? 0 : (dx / std::abs(dx));
    dy = (dy == 0) ? 0 : (dy / std::abs(dy));

    // Find the index in the directions array
    for (size_t i = 0; i < directions.size(); ++i) {
        if (directions[i].first == dx && directions[i].second == dy) {
            return i;
        }
    }
    return -1; // This should never happen
}

//
// Function to generate the navigation map
//
NavGrid generateNavigationGrid(const Graph& graph) {
	return generateNavigationGrid(graph.infoGrid, graph.baseEdges, graph.abstractEdges,
			graph.baseNodes, graph.abstractNodes);
}

// Compute the granular angle between two points
double computeAngle(const Point& from, const Point& to) {
    int dx = to.first - from.first;
    int dy = to.second - from.second;
    double angle = std::atan2(dy, dx) * 180.0 / MY_PI;
    if (angle < 0) angle += 360; // Normalize angle to [0, 360)
    return angle;
}

NavGrid generateNavigationGrid(
    const Grid& grid,
    const std::vector<Edge>& baseEdges,
    const std::vector<AbstractEdge>& abstractEdges,
    const std::vector<Point>& baseNodes,
    const std::vector<AbstractNode>& abstractNodes)
{
    // Dimensions of the grid
    int rows = grid.size();
    int cols = grid[0].size();

    // Initialize the navigation map
    NavGrid navMap(rows, std::vector<GridPointInfo>(cols));

    // For every empty grid cell (1 = path), calculate navigation info
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c] != 1) continue; // Skip walls or non-path cells

            Point current = {r, c};

            // Find the closest base edge using its path field
            int closestBaseEdgeIdx = -1;
            double minDistanceBase = std::numeric_limits<double>::max();
            for (size_t i = 0; i < baseEdges.size(); ++i) {
                for (const auto& point : baseEdges[i].path) {
                    double distance = std::hypot(current.first - point.first, current.second - point.second);
                    if (distance < minDistanceBase) {
                        minDistanceBase = distance;
                        closestBaseEdgeIdx = i;
                    }
                }
            }

            // Find the closest abstract edge using its path field
            int closestAbstractEdgeIdx = -1;
            double minDistanceAbstract = std::numeric_limits<double>::max();
            for (size_t i = 0; i < abstractEdges.size(); ++i) {
                for (const auto& point : abstractEdges[i].path) {
                    double distance = std::hypot(current.first - point.first, current.second - point.second);
                    if (distance < minDistanceAbstract) {
                        minDistanceAbstract = distance;
                        closestAbstractEdgeIdx = i;
                    }
                }
            }

            navMap[r][c].closestBaseEdgeIdx = closestBaseEdgeIdx;
            navMap[r][c].closestAbstractEdgeIdx = closestAbstractEdgeIdx;

            // Compute next step directions using abstract edge paths
            const auto& abstractEdge = abstractEdges[closestAbstractEdgeIdx];

            // Calculate directions and angles towards the "from" and "to" nodes of the abstract edge
            Point fromNode = baseNodes[abstractNodes[abstractEdge.from].baseCenterNode];
            Point toNode = baseNodes[abstractNodes[abstractEdge.to].baseCenterNode];

            navMap[r][c].directionToFromNode = computeDirection(current, fromNode);
            navMap[r][c].directionToToNode = computeDirection(current, toNode);

            navMap[r][c].angleToFromNode = computeAngle(current, fromNode);
            navMap[r][c].angleToToNode = computeAngle(current, toNode);
        }
    }

    return navMap;
}

//////////////////////////////////////////////////////////////////////

PathCostMap computeAllPaths(const std::vector<Edge>& baseEdges, int numNodes)
{
    // Create an adjacency list from the edges
    std::vector<std::vector<std::pair<int, int>>> adjList(numNodes);
    for (const auto& edge : baseEdges) {
        adjList[edge.from].emplace_back(edge.to, edge.path.size());
        adjList[edge.to].emplace_back(edge.from, edge.path.size()); // Assuming undirected graph
    }

    PathCostMap costs;
    const int INF = std::numeric_limits<int>::max();

    // Run Dijkstra's algorithm for each node as the source
    std::cerr << "==COMPUTE PATHS for nodes:" << numNodes << std::endl;
    for (int start = 0; start < numNodes; ++start) {
    	std::cerr << "  NODE " << start << std::endl;
        // Distance vector initialized to infinity
        std::vector<int> dist(numNodes, INF);
        dist[start] = 0;

        // Min-heap priority queue: {distance, node}
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
        pq.push({0, start});

        while (!pq.empty()) {
            auto [currentDist, currentNode] = pq.top();
            pq.pop();

            // If this distance is already outdated, skip it
            if (currentDist > dist[currentNode]) {
                continue;
            }

            // Explore neighbors
            std::cerr << "  GET PQ for adjacency: " << adjList[currentNode].size();
            for (const auto& [neighbor, weight] : adjList[currentNode]) {
                if (dist[currentNode] + weight < dist[neighbor]) {
                    dist[neighbor] = dist[currentNode] + weight;
                    pq.push({dist[neighbor], neighbor});
                }
            }
            std::cerr << "  PQ => " << pq.size() << std::endl;
        }

        // Store results for this source node
        for (int end = 0; end < numNodes; ++end) {
            if (dist[end] != INF) {
                costs[{start, end}] = dist[end];
            }
        }
    }
    std::cerr << "==MADE PATH COST MAP: " << costs.size() << std::endl;
    return costs;
}


////////////////////////////////////////////////////////

Graph makeGraph(const Grid& floorGrid)
{
	makeTGA("GRID.tga", floorGrid, 0xff);

	//
	// Need to copy the floorGrid since ZSThinning removes floor
	//
	Graph graph;
	graph.infoGrid = floorGrid;

	//
	// Thin the floor down to a single path
	//
    Algo::ZSThinning(graph.infoGrid);
	makeTGA("THIN.tga", graph.infoGrid);

	//
	// Get the deadEnds and Nodes
	//
	graph.deadEnds = detectDeadEnds(graph.infoGrid);
	graph.baseNodes = detectNodes(graph.infoGrid);

	//
	// Set NODE and DEND in infoGrid and find Edges
	//
	updateInfoGrid(graph.infoGrid, graph.baseNodes, graph.deadEnds);
	graph.baseEdges = findEdges(graph.infoGrid, graph.baseNodes, graph.deadEnds);
	makeTGA("GRAPH.tga", graph.infoGrid);

	//
	// Restore Walls (since thinning make FLOOR = EMPTY, then expand Paths to fill EMPTY
	//
	restoreWalls(graph.infoGrid, floorGrid);
	thickenPaths(graph.infoGrid);
	makeTGA("XPND.tga", graph.infoGrid);

	//
	// Make the higher level abstract graph of nodes and edges
	//
	std::tie(graph.abstractNodes, graph.abstractEdges) = createAbstractGraph(graph.baseNodes, graph.deadEnds, graph.baseEdges, 5.0, 2);

	graph.navGrid = generateNavigationGrid(graph);

	std::cerr << "==COMPUTE ALL PATHS" << std::endl;
	graph.pathCostMap = computeAllPaths(graph.baseEdges, graph.baseNodes.size());
	std::cerr << "==MADE GRAPH" << std::endl;

	debugDump(graph);
	return graph;
}


///////////////////////////////////////////////////////////

void debugDump(const Graph& graph)
{
	Grid tempGrid(graph.infoGrid);

	std::cerr << "\nAbstract Nodes:  " << graph.abstractNodes.size() << std::endl;
	for (const auto& node : graph.abstractNodes) {
		std::cerr << "Cluster " << node.id << " Center: (" << node.center.first << ", " << node.center.second << ")"
				<< " size dend:" << node.baseDeadEnds.size() << " node: " << node.baseNodes.size() << std::endl;
	}

	std::cerr << "\nAbstract Edges: " << graph.abstractEdges.size() << std::endl;
	for (const auto& edge : graph.abstractEdges) {
		std::cerr << "Edge from Cluster " << edge.from << " to Cluster " << edge.to << " with cost " << edge.path.size() << std::endl;
	}
	for (const auto& e : graph.abstractEdges) {
		std::cerr << "EDGE: " << e.from << " to " << e.to << " p: " << e.path.size() << std::endl;
		const auto& f = graph.abstractNodes[e.from];
		const auto& t = graph.abstractNodes[e.to];
		std::cerr << "POINT: " << f.center.first << "," << f.center.second << " to: " << t.center.first << "," << t.center.second << std::endl;
		for (const auto& p : e.path) {
			tempGrid[p.second][p.first] = 0x05;
		}
		if (f.baseCenterNode != -1) {
			tempGrid[ graph.baseNodes[f.baseCenterNode].second] [graph.baseNodes[f.baseCenterNode].first] = 0x06;
		}
		if (t.baseCenterNode != -1) {
			tempGrid[graph.baseNodes[t.baseCenterNode].second] [graph.baseNodes[t.baseCenterNode].first] = 0x06;
		}
	}
	std::cerr << "NODES: " << graph.baseNodes.size() << std::endl
			<< "DENDS: " << graph.deadEnds.size() << std::endl
			<< "EDGES: " << graph.baseEdges.size() << std::endl
			<< std::endl
			<< "ABNOD: " << graph.abstractNodes.size() << std::endl
			<< "ABEDG: " << graph.abstractEdges.size() << std::endl;

	makeTGA("META.tga", tempGrid);
}

} // namespace
