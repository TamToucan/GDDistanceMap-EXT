#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <unordered_set>

#include "ZSThinning.hpp"
#include "GridToGraph.hpp"

#include "TGA.hpp"

using Pattern = std::vector<std::vector<int>>;

namespace {

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
			if (grid[y+i][x+j]) {
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

std::vector<Edge> findEdges(const Grid& grid, const std::vector<Point>& nodes, const std::vector<Point>& deadEnds) {
    // Direction vectors for moving: up, down, left, right
    const std::vector<Point> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    // Store edges as pairs of points (start and end of an edge)
    std::vector<Edge> edges;

    // Convert nodes and deadEnds into a set for quick lookup
    std::unordered_set<Point, PairHash> importantPoints(nodes.begin(), nodes.end());
    importantPoints.insert(deadEnds.begin(), deadEnds.end());

    // Keep track of visited points to avoid revisiting
    Grid visited(grid[0].size(), std::vector<int>(grid.size(), 0));

    // Lambda function to check if a point is within grid bounds
    auto inBounds = [&](int x, int y) {
    	std::cerr << "IN: " << x << "," << y << std::endl;
        return x >= 0 && x < grid[0].size() && y >= 0 && y < grid.size();
    };

    // Function to traverse a path from a given start point
    auto traversePath = [&](const Point& start) {
        std::vector<Point> path; // To store the path
        Point current = start;

    	std::cerr << "==TRAVERSE: " << start.first << "," << start.second << std::endl;
        // Perform a step-by-step traversal
        while (true) {
            visited[current.second][current.first] = 1;
            path.push_back(current);
            std::cerr << "  ADD:" << current.first << "," << current.second << std::endl;

            Point next(-1, -1); // Next point to move to
            int validNeighbors = 0;

            // Check all four directions
            for (const auto& dir : directions) {
                int nx = current.first + dir.first;
                int ny = current.second + dir.second;
                std::cerr << "  NEXT: " << nx << "," << ny << std::endl;
                if (inBounds(nx, ny) && !visited[ny][nx] && grid[ny][nx] == 1) {
                    validNeighbors++;
                    if (validNeighbors == 1) {
                        next = {nx, ny};
                    } else {
                        // More than one valid neighbor means we are at a node or intersection
                        break;
                    }
                }
            }

            // If multiple neighbors or an important point, stop the traversal
            if (validNeighbors != 1 || importantPoints.count(current)) {
                break;
                std::cerr << "  BREAK:" << path.size() << std::endl;
            }

            current = next; // Continue to the next point
        }

        return path;
    };

    // Traverse from every node and dead end
    std::cerr << "============TRAVERSE JUNCS:" << std::endl;
    for (const auto& start : nodes) {
        if (visited[start.second][start.first]) continue;

        for (const auto& dir : directions) {
            int nx = start.first + dir.first;
            int ny = start.second + dir.second;

            if (inBounds(nx, ny) && !visited[ny][nx] && grid[ny][nx] == 1) {
                auto path = traversePath({nx, ny});
                if (!path.empty()) {
                	std::cerr << "=== => STORE PATH:" << path.size() << " edges:" << edges.size() << std::endl;
                    edges.emplace_back(start, path.back());
                }
            }
        }
    }

    std::cerr << "============TRAVERSE DEAD:" << std::endl;
    for (const auto& start : deadEnds) {
        if (visited[start.second][start.first]) continue;

        for (const auto& dir : directions) {
            int nx = start.first + dir.first;
            int ny = start.second + dir.second;

            if (inBounds(nx, ny) && !visited[ny][nx] && grid[ny][nx] == 1) {
                auto path = traversePath({nx, ny});
                if (!path.empty()) {
                	std::cerr << "=== => STORE PATH:" << path.size() << " edges:" << edges.size() << std::endl;
                    edges.emplace_back(start, path.back());
                }
            }
        }
    }

    return edges;
}

///////////////////////////////////////////////////////////

namespace {
	void makeTGA(const char* name, Grid& grid, unsigned char forZero=0x00)
	{
		unsigned char* pPixel = new unsigned char[grid.size()*grid[0].size()*4];
		unsigned char* pData = pPixel;
		for (int y=grid.size()-1; y>= 0; --y) {
			for (int xy : grid[y]) {
				switch (xy)
				{
				case 0x02:
					*pData++ = 0xff;
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				case 0x03:
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0xff;
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

void makeGraph(Grid& floorGrid)
{
	makeTGA("GRID.tga", floorGrid, 0xff);
	std::cerr << "THIN IT" << std::endl;
    Algo::ZSThinning(floorGrid);
	std::cerr << "THINNED" << std::endl;

	makeTGA("THIN.tga", floorGrid);
	std::cerr << "GET DEAD" << std::endl;
	std::vector<Point> deadends = detectDeadEnds(floorGrid);
	std::cerr << "GET NODES" << std::endl;
	std::vector<Point> nodes = detectNodes(floorGrid);
	std::cerr << "GET EDGES" << std::endl;
	std::vector<Edge> edges = findEdges(floorGrid, nodes, deadends);

	std::cerr << "UPDATE GRID: " << deadends.size() << " " << nodes.size() << std::endl;
	for (Point p : deadends) {
		floorGrid[p.second][p.first] = 0x2;
	}
	for (Point p : nodes) {
		floorGrid[p.second][p.first] = 0x3;
	}
	makeTGA("GRAPH.tga", floorGrid);
}

///////////////////////////////////////////////////////////

}
