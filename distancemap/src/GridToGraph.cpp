#include <iostream>
#include <vector>
#include <algorithm>
#include <tuple>
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

using namespace GridType;

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
// all EMPTY and 1 PATH since PATH==1 and EMPTY==0 and all others > 1.
//
inline
int countNeighbors(const Grid& grid, int x, int y) {
	return grid[y-1][x-1]
		+ grid[y-1][x]
		+ grid[y-1][x+1]

		+ grid[y][x-1]
		+ grid[y][x+1]

		+ grid[y+1][x-1]
		+ grid[y+1][x]
		+ grid[y+1][x+1];
}

} // anonymouse

////////////////////////////////////////////////////////////////////////////////////////


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
            	// Get the ID from the 9 (3x3) bits and check if a pattern existsnodes.size() << " = " << x+1 << "," << y+1 << std::endl;
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
				switch ((xy&0xffff)>>16)
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

/////////////////////////////////////////////////////////////////////////////////////////

//
// Set the Node and DeadEnds
//
void markGridNodes(Grid& grid, const std::vector<Point>& nodes, const std::vector<Point>& deadEnds) {
	int nodeIdx = GridToGraph::NODE;
	int dendIdx = GridToGraph::DEND;
    for (const auto& n : nodes) {
    	grid[n.second][n.first] = nodeIdx++;
    }
    for (const auto& de : deadEnds) {
    	grid[de.second][de.first] = dendIdx++;
    }
}

//
// Set the Paths to the edge index
//
void markGridEdges(Grid& grid, const std::vector<Edge>& edges) {
	int edgeIdx = GridToGraph::EDGE;
    for (const auto& e : edges) {
    	for (const auto& p : e.path) {
    		if (grid[p.second][p.first] == GridToGraph::PATH) {
    			grid[p.second][p.first] = edgeIdx;
    		}
    	}
    	++edgeIdx;
    }
}

void markGridBoundaries(Grid& grid) {
    int rows = grid.size();
    int cols = grid[0].size();

    for (int r = 1; r < rows-1; ++r) {
        for (int c = 1; c < cols-1; ++c) {
            if (grid[r][c]&WALL) {
            	std::cerr << "...GOT BOUND " << c << " " << r << std::endl;

            	for (const auto& [dr, dc] : GridType::directions) {
            		int nr = r + dr, nc = c + dc;

            		if (grid[nr][nc]&(WALL|BOUNDARY)) continue;

            		grid[r][c] &= ~WALL;  // Mark as BOUNDARY
            		grid[r][c] |= BOUNDARY;  // Mark as BOUNDARY
            		std::cerr << "#BOUND " << c << " " << r << std::hex << grid[r][c] << std::dec << std::endl;
            		break;  // No need to check further
            	}
            }
        }
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

#if 0
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
            if (grid[r][c]&GridToGraph::EDGE) {
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
                grid[nr][nc] = GridToGraph::XPND | sizeOfPath;
                queue.push({nr, nc});
            }
        }
    }
}
#else
void thickenPaths(Grid& grid)
{
    int rows = grid.size();
    int cols = grid[0].size();

    std::queue<std::pair<Point, int>> queue;  // {position, distance}

    // Initialize queue with all EDGE points from infoGrid
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c] & GridToGraph::EDGE) {  // Only EDGE points start BFS
                queue.push({{r, c}, 0});
            }
        }
    }

    // Process BFS queue
    while (!queue.empty()) {
        auto [pos, dist] = queue.front();
        queue.pop();

        int r = pos.first, c = pos.second;

        // Attempt to expand in all 8 directions
        for (const auto& [dr, dc] : directions) {
            int nr = r + dr;
            int nc = c + dc;

            // Check bounds and ensure we only expand empty cells
            if (grid[nr][nc] == GridToGraph::EMPTY) {
                int newDist = dist + 1;

                grid[nr][nc] = (newDist & 0xFFFF) | GridToGraph::XPND;
                queue.push({{nr, nc}, newDist});
            }
        }
    }
}

#endif


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
    std::set<AbstractEdge> abstractEdgesSet;
    for (int edgeIdx=0; edgeIdx < edges.size(); ++edgeIdx) {
    	const auto& edge = edges[edgeIdx];
        int clusterFrom = clusterLabels[edge.from];
        int clusterTo = edge.toDeadEnd ? clusterLabels[nodes.size() + edge.to]
                                       : clusterLabels[edge.to];

        if (clusterFrom < 0 || clusterTo < 0 || clusterFrom == clusterTo) {
            continue; // Skip edges involving noise or intra-cluster edges
        }

        double cost = edge.path.size();
        abstractEdgesSet.insert({clusterFrom, clusterTo });
    }
    // Turn set into unique vector
    std::vector<AbstractEdge> abstractEdges(abstractEdgesSet.begin(), abstractEdgesSet.end());

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

    std::cerr << "============ NAV GRID"  << std::endl;
    // Initialize the navigation map
    NavGrid navMap(rows, std::vector<GridPointInfo>(cols));

    // For every empty grid cell calculate navigation info
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int t = grid[r][c];
            if (t&WALL) {
            	continue; // Skip walls
            }

            Point current = {c, r};

            // Find the closest base edge using its path field
            int closestBaseEdgeIdx = -1;
            double minDistanceBase = std::numeric_limits<double>::max();
            for (size_t i = 0; i < baseEdges.size(); ++i) {
                if (baseEdges[i].toDeadEnd) continue;
                for (const auto& point : baseEdges[i].path) {
                    double distance = std::hypot(current.first - point.first, current.second - point.second);
                    if (distance < minDistanceBase) {
                        minDistanceBase = distance;
                        closestBaseEdgeIdx = i;
                    }
                }
            }
            if (closestBaseEdgeIdx == -1) {
            	std::cerr << "ERROR: no edge found" << std::endl;
            }

            // Find which baseNode of edgeis closer
            const auto& closeEdge = baseEdges[closestBaseEdgeIdx];
            const auto& closePath = closeEdge.path;
            Point fromPnt = closePath[0];
            Point toPnt = closePath[closePath.size()-1];
            double fromDist = std::hypot(current.first - fromPnt.first, current.second - fromPnt.second);
            double toDist = std::hypot(current.first - toPnt.first, current.second - toPnt.second);
            std::cerr << " " << c << "," << r << " bEdge:" << closestBaseEdgeIdx << std::endl;
            std::cerr << "   " << " fidx:" << closeEdge.from << " tidx:" << closeEdge.to << std::endl;
            std::cerr << "   " << " f:" << fromPnt.first << "," << fromPnt.second << " d:" << fromDist << std::endl;
            std::cerr << "   " << " t:" << toPnt.first << "," << toPnt.second << " d:" << toDist << std::endl;
            int closestBaseNodeIdx = fromDist < toDist
            		? closeEdge.from
            		: closeEdge.to;
            std::cerr << "   " << c << "," << r << " close base: " << closestBaseNodeIdx << " t:" << toPnt.first << "," << toPnt.second << " d:" << toDist << std::endl;

            // Find the closest abstract edge using its path field
            int closestAbstractEdgeIdx = -1;
            double minDistanceAbstract = std::numeric_limits<double>::max();
            for (size_t i = 0; i < abstractEdges.size(); ++i) {
                for (const auto& point : abstractEdges[i].path) {
                    double distance = std::hypot(current.first - point.first, current.second - point.second);
                    if (distance < minDistanceAbstract) {
                        minDistanceAbstract = distance;
                        closestAbstractEdgeIdx = i;
                        std::cerr << "   p:" << point.first << "," << point.second << " dist:" << distance
                        		<< " => closest:" << closestAbstractEdgeIdx << std::endl;
                    }
                }
            }
            if (closestAbstractEdgeIdx == -1) {
            	std::cerr << "ERROR: no ABedge found" << std::endl;
            }

            navMap[r][c].closestBaseNodeIdx = closestBaseNodeIdx;
            navMap[r][c].closestBaseEdgeIdx = closestBaseEdgeIdx;
            navMap[r][c].closestAbstractEdgeIdx = closestAbstractEdgeIdx;

            // Find which of the 2 abstract nodes is closest
            const auto& abNodeFromIdx = abstractEdges[closestAbstractEdgeIdx].from;
            const auto& abNodeToIdx = abstractEdges[closestAbstractEdgeIdx].to;
            const auto& baseFrom = baseNodes[ abstractNodes[ abNodeFromIdx ].baseCenterNode ];
            const auto& baseTo = baseNodes[ abstractNodes[ abNodeToIdx ].baseCenterNode ];
            double distFrom = std::hypot(current.first - baseFrom.first, current.second - baseFrom.second);
            double distTo = std::hypot(current.first - baseTo.first, current.second - baseTo.second);
            navMap[r][c].closestAbstractNodeIdx = distFrom < distTo ? abNodeFromIdx : abNodeToIdx;

            // Compute next step directions using abstract edge paths
            const auto& abstractEdge = abstractEdges[closestAbstractEdgeIdx];

            // Calculate directions and angles towards the "from" and "to" nodes of the abstract edge
            Point fromNode = baseNodes[abstractNodes[abstractEdge.from].baseCenterNode];
            Point toNode = baseNodes[abstractNodes[abstractEdge.to].baseCenterNode];

            navMap[r][c].directionToFromNode = computeDirection(current, fromNode);
            navMap[r][c].directionToToNode = computeDirection(current, toNode);

            navMap[r][c].angleToFromNode = computeAngle(current, fromNode);
            navMap[r][c].angleToToNode = computeAngle(current, toNode);
            std::cerr << "  AT] " << c << "," << r << " f:" << fromNode.first << "," << fromNode.second << " t:" << toNode.first << "," << toNode.second << std::endl;
        }
    }

    return navMap;
}

//////////////////////////////////////////////////////////////////////

PathCostMap computeAllPaths(const std::vector<Edge>& baseEdges, int numNodes)
{
    // Create an adjacency list from the edges
	std::cerr << "==INIT ALL PATHS: " << numNodes << std::endl;
    std::vector<std::vector<std::pair<int, int>>> adjList(numNodes);
    for (const auto& edge : baseEdges) {
        adjList[edge.from].emplace_back(edge.to, edge.path.size());
        adjList[edge.to].emplace_back(edge.from, edge.path.size()); // Assuming undirected graph
        std::cerr << "  EDGE FROM: " << edge.from << " to: " << edge.to << " dend:" << edge.toDeadEnd << std::endl;
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
            for (const auto& [neighbor, weight] : adjList[currentNode]) {
                if (dist[currentNode] + weight < dist[neighbor]) {
                    dist[neighbor] = dist[currentNode] + weight;
                    pq.push({dist[neighbor], neighbor});
                }
            }
        }

        // Store results for this source node
        for (int end = 0; end < numNodes; ++end) {
            if (dist[end] != INF) {
                costs[{start, end}] = dist[end];
            }
        }
    }
    std::cerr << "==MADE PATH COST MAP: " << costs.size() << std::endl;
    for (const auto& e : costs) {
    	std::cerr << "  " << e.first.first << "," << e.first.first << " => " << e.second << std::endl;
    }
    return costs;
}

////////////////////////////////////////////////////////

BaseGraph generateBaseGraph(const std::vector<Edge>& baseEdges)
{
	BaseGraph baseGraph;
    baseGraph.reserve(baseEdges.size()); // Reserve space for efficiency

    for (const auto& edge : baseEdges) {
        if (edge.toDeadEnd) continue;
        int from = edge.from;
        int to = edge.to;
        int cost = edge.path.size(); // Use path length as edge cost

        // Add edge in both directions (if undirected)
        baseGraph[from].emplace_back(to, cost);
        baseGraph[to].emplace_back(from, cost);
        std::cerr << "BASE GRAPH: " << from << " -> " << to << " c: " << cost << std::endl;
    }
    return baseGraph;
}

////////////////////////////////////////////////////////

FallbackGrid generateFallbackGrid(const Graph& graph)
{
	std::cerr << "################ GEN FBACK ############" << std::endl;

	FallbackGrid fallbackGrid(graph.infoGrid.size(), std::vector<FallbackCell>(graph.infoGrid[0].size()));
	//
	// Make a grid of all points covered by flowFields
	//
	Grid flowFieldCoverage(graph.infoGrid.size(), std::vector<int>(graph.infoGrid[0].size(), 0));
	const int rows = flowFieldCoverage.size();
	const int cols = flowFieldCoverage[0].size();
	for (const auto& ff : graph.flowFields) {
		for (const auto& fd : ff.flowData) {
			const Point& pnt = fd.first;
			flowFieldCoverage[pnt.second][pnt.first] = 1;
		}
	}

	// Set the walls as "covered" (but use -1 to differentiate)
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			if (graph.infoGrid[y][x]&WALL) {
				flowFieldCoverage[y][x] = -1;
			}
		}
	}

	std::cerr << std::endl;
	for (int x = 0; x < cols; ++x) {
		std::cerr << x%10;
	}
	std::cerr << std::endl;
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			if (flowFieldCoverage[y][x] > 0) {
				std::cerr << "X";
			} else if (flowFieldCoverage[y][x] < 0) {
				std::cerr << "#";
			}
			else {
				std::cerr << " ";
			}
		}
		std::cerr << "  " << y << std::endl;
	}
	std::cerr << std::endl;

    // Find boundary points (cells adjacent to flow fields)
	std::queue<std::tuple<int, int, int, int>> queue; // {x, y, nextFlowX, nextFlowY
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (flowFieldCoverage[r][c] == 1) {  // Flow field covered point
                for (const auto& [dr, dc] : directions) {
                    int nr = r + dr, nc = c + dc;

                    if (flowFieldCoverage[nr][nc] == 0) {
                        queue.emplace(nr, nc, r, c);  // Add to queue as boundary point
                        fallbackGrid[nr][nc] = {r, c, 1};  // Distance = 1
                    }
                }
            }
        }
    }

	// Expand outward from boundary cells using BFS
    while (!queue.empty()) {
        auto [r, c, targetX, targetY] = queue.front();
        queue.pop();

        for (const auto& [dr, dc] : directions) {
            int nr = r + dr, nc = c + dc;

            if (fallbackGrid[nr][nc].distance != -1 || flowFieldCoverage[nr][nc] != 0) {
                continue;  // Skip walls & already processed points
            }

            // Assign new fallback cell
            fallbackGrid[nr][nc] = {targetX, targetY, fallbackGrid[r][c].distance + 1};
            queue.emplace(nr, nc, targetX, targetY);  // Expand further
        }
    }
	return fallbackGrid;
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
	markGridNodes(graph.infoGrid, graph.baseNodes, graph.deadEnds);
	graph.baseEdges = findEdges(graph.infoGrid, graph.baseNodes, graph.deadEnds);
	markGridEdges(graph.infoGrid, graph.baseEdges);
	makeTGA("GRAPH.tga", graph.infoGrid);

	//
	// Restore Walls (since thinning make FLOOR = EMPTY, then expand Paths to fill EMPTY
	//
	restoreWalls(graph.infoGrid, floorGrid);
	markGridBoundaries(graph.infoGrid);
	thickenPaths(graph.infoGrid);
	makeTGA("XPND.tga", graph.infoGrid);

	//
	// Make the higher level abstract graph of nodes and edges
	//
	std::tie(graph.abstractNodes, graph.abstractEdges) = createAbstractGraph(graph.baseNodes, graph.deadEnds, graph.baseEdges, 5.0, 2);

	graph.navGrid = generateNavigationGrid(graph);
	graph.baseGraph = generateBaseGraph(graph.baseEdges);

	//
	// Generate sparse flow fields for all the abstract edges
	// and the fallback graph to use when no flow field
	//
    FlowField::generateFlowFieldsParallel(&graph, graph.flowFields);
	graph.fallbackGrid = generateFallbackGrid(graph);

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
		std::cerr << "  EDGE: " << e.from << " to " << e.to << " p: " << e.path.size() << std::endl;
		const auto& f = graph.abstractNodes[e.from];
		const auto& t = graph.abstractNodes[e.to];
		std::cerr << "    CENTER: " << f.center.first << "," << f.center.second << " to: " << t.center.first << "," << t.center.second << std::endl;
		std::cerr << "    BASE F:" << f.baseCenterNode << " FROM: " << graph.baseNodes[f.baseCenterNode].first << "," << graph.baseNodes[f.baseCenterNode].second << std::endl;
		std::cerr << "    BASE T:" << t.baseCenterNode << " TO  : " << graph.baseNodes[t.baseCenterNode].first << "," << graph.baseNodes[t.baseCenterNode].second << std::endl;
		for (const auto& p : e.path) {
			tempGrid[p.second][p.first] = 0x05;
			std::cerr << "      " << p.first << "," << p.second << std::endl;
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

	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		if (col > 9) std::cerr << col << "  ";
		else std::cerr << col << "   ";
	}
	std::cerr << std::endl;
	std::cerr << "INFO GRID (nodes/edges)" << std::endl;
	for (int row=0; row < graph.infoGrid.size(); ++row) {

		if (row > 9) std::cerr << row << "  " << std::endl;
		else std::cerr << row << "   " << std::endl;

		for (int col=0; col < graph.infoGrid[0].size(); ++col) {
			int n = graph.infoGrid[row][col];
			if (n&NODE) {
				n &= 0xffff;
				if (n >= 10) {
					std::cerr << n << "n ";
				}
				else if (n >= 0) {
					std::cerr << n << "n  ";
				}
				else {
					std::cerr << "--- ";
				}
			}
			else if (n&EDGE) {
				int e = n & 0xffff;
				if (e >= 100) {
					std::cerr << e << "e";
				} else if (e >= 10) {
					std::cerr << e << "e ";
				}
				else if (e >= 0) {
					std::cerr << e << "e  ";
				}
				else {
					std::cerr << "--- ";
				}
			}
			else if (n&XPND) {
				int c = n & 0xffff;
				if (c >= 100) {
					std::cerr << c << "c";
				} else if (c >= 10) {
					std::cerr << c << "c ";
				}
				else if (c >= 0) {
					std::cerr << c << "c  ";
				}
				else {
					std::cerr << "--- ";
				}
			}
			else {
				std::cerr << "--- ";
			}
		}
		std::cerr << " #" << std::endl;
	}
	std::cerr << "NAV GIRD (closet abstract/base)" << std::endl;
	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		if (col > 9) std::cerr << col << "   ";
		else std::cerr << col << "    ";
	}
	std::cerr << std::endl;
	for (int row=0; row < graph.infoGrid.size(); ++row) {

		if (row > 9) std::cerr << row << "   " << std::endl;
		else std::cerr << row << "    " << std::endl;

		for (int col=0; col < graph.infoGrid[0].size(); ++col) {
			if (graph.infoGrid[row][col]&WALL) {
				std::cerr << "---- ";
			}
			else {
				const auto& info = graph.navGrid[row][col];
				if (info.closestBaseNodeIdx >= 10) {
					std::cerr << info.closestAbstractNodeIdx << "/" << info.closestBaseNodeIdx << " ";
				}
				else {
					std::cerr << info.closestAbstractNodeIdx << "/" << info.closestBaseNodeIdx << "  ";
				}
			}
		}
		std::cerr << " #" << std::endl;
	}

	std::cerr << "ABSTRACT NODES (idx / base center)" << std::endl;
	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		if (col > 9) std::cerr << col << "   ";
		else std::cerr << col << "    ";
	}
	for (int row=0; row < graph.infoGrid.size(); ++row) {

		if (row > 9) std::cerr << row << "   " << std::endl;
		else std::cerr << row << "    " << std::endl;

		for (int col=0; col < graph.infoGrid[0].size(); ++col) {
			bool found = false;
			int bidx = 0;
			for (const auto& n : graph.abstractNodes) {
				const auto& p = graph.baseNodes[n.baseCenterNode];
				if (p.first == col && p.second == row) {
					int b = n.baseCenterNode;
					if (b >= 10) {
						std::cerr << bidx << "/" << b << " ";
					}
					else {
						std::cerr << bidx << "/" << b << "  ";
					}
					found = true;
					break;
				}
				++bidx;
			}
			if (!found) {
				int eidx=0;
				for (const auto& e : graph.abstractEdges) {
					for (const auto& p : e.path) {
						if (p.first == col && p.second == row) {
							if (eidx >= 10) {
								std::cerr << eidx << "e  ";
							}
							else {
								std::cerr << eidx << "e   ";
							}
							found = true;
							break;
						}
					}
					if (found) break;
					++eidx;
				}
			}
			if (!found) {
				std::cerr << "---- ";
			}
		}
		std::cerr << " #" << std::endl;
	}

	std::cerr << "FLOW FIELDS (dir idx) " << std::endl;
	int abEdge=0;
	for (const auto ff : graph.flowFields) {
		std::cerr << "FOR ABEDGE: " << abEdge++ << std::endl;
		std::cerr << "   ";
		for (int col=0; col < graph.infoGrid[0].size(); ++col) {
			if (col > 9) std::cerr << col << " ";
			else std::cerr << col << "  ";
		}
		std::cerr << std::endl;
		for (int row=0; row < graph.infoGrid.size(); ++row) {
			if (row > 9) std::cerr << row << " ";
			else std::cerr << row << "  ";
			for (int col=0; col < graph.infoGrid[0].size(); ++col) {
				const auto& dirNcost = ff.unpack({col, row});
				if (dirNcost) {
					std::cerr << dirNcost->first << "  ";
				}
				else {
					std::cerr << "-- ";
				}
			}
		std::cerr << std::endl;
		}
	}


	std::cerr << "FALLBACK GRAPH " << std::endl;
	for (int row=0; row < graph.fallbackGrid.size(); ++row) {
		for (int col=0; col < graph.fallbackGrid[0].size(); ++col) {
			const auto& cell = graph.fallbackGrid[row][col];
			if (cell.distance != -1) {
				std::cerr << std::showpos << cell.nextFlowX << "," << std::showpos << cell.nextFlowY << "  ";
			}
			else {
				std::cerr << "     " << "  ";
			}
		}
		std::cerr << std::endl;
	}
	std::cerr << std::noshowpos;
	for (int col=0; col < graph.fallbackGrid[0].size(); ++col) {
		if (col%10) {
			std::cerr << "    ";
		}
		else {
			std::cerr << " " << (col/10) << "   ";
		}
	}
	std::cerr << std::endl;
	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		std::cerr << " " << (col%10) << "   ";
	}
	std::cerr << std::endl;
	for (int row=0; row < graph.fallbackGrid.size(); ++row) {
		for (int col=0; col < graph.fallbackGrid[0].size(); ++col) {
			const auto& cell = graph.fallbackGrid[row][col];
			std::string chr = "    ";
			int dx = cell.nextFlowX == col ? 0
					: cell.nextFlowX > col ? 1
					: -1;
			int dy = cell.nextFlowY == row ? 0
					: cell.nextFlowY > row ? 1
					: -1;


			if (cell.distance != -1) {
				switch (dx) {
				case -1: {
					switch (dy) {
					case -1: chr = " NW "; break;
					case 0:  chr = " W  "; break;
					case +1: chr = " SW "; break;
					}
				}
				break;
				case 0: {
					switch (dy) {
					case -1: chr = " N  "; break;
					case 0:  chr = " .. "; break;
					case +1: chr = " S  "; break;
					}
				}
				break;
				case +1: {
					switch (dy) {
					case -1: chr = " NE "; break;
					case 0:  chr = " E  "; break;
					case +1: chr = " SE "; break;
					}
					break;
				}
				}
			}
			std::cerr << chr;
		}
		std::cerr << " " << row << std::endl;
	}

	std::cerr << "WALLS" << std::endl;
	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		if (col%10) {
			std::cerr << " ";
		}
		else {
			std::cerr << (col/10);
		}
	}
	std::cerr << std::endl;
	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		std::cerr << (col%10);
	}
	std::cerr << std::endl;
	for (int row=0; row < graph.infoGrid.size(); ++row) {
		for (int col=0; col < graph.infoGrid[0].size(); ++col) {
			std::cerr << ((graph.infoGrid[row][col]&WALL) ? "#" : " ");
		}
		std::cerr << " " << row << std::endl;
	}
}

} // namespace
