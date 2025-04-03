#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <tuple>
#include <queue>
#include <unordered_set>
#include <deque>
#include <utility>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <map>
#include <set>

#include "GridToGraph.hpp"

#include <string>

#include "AbstractMST.hpp"
#include "gdextension_interface.h"
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
    // 			-#-
    //			###
    //			-#-
    patterns.push_back({
        {0, 1, 0 },
        {1, 1, 1 },
        {0, 1, 0 },
    });

    // 4-Way Junction (X-Junction)
    // 			#-#
    //			-#-
    //			#-#
    patterns.push_back({
        {1, 0, 1 },
        {0, 1, 0 },
        {1, 0, 1 },
    });
    // 1-Way Junction (X-Junction)
    // 			#-#
    //			-#-
    //			#--
    patterns.push_back({
        {1, 0, 1 },
        {0, 1, 0 },
        {1, 0, 0 },
    });
    // 1-Way Junction (X-Junction) 1 filled
    // 			#-#
    //			##-
    //			#--
    patterns.push_back({
        {1, 0, 1 },
        {1, 1, 0 },
        {1, 0, 0 },
    });
    // T-Junction
    // 			-#-
    //			###
    //			---
    patterns.push_back({
        {0, 1, 0 },
        {1, 1, 1 },
        {0, 0, 0 },
    });
    // T-junc with 1 diag
    // 			-#-
    //			-##
    //			#--
    patterns.push_back({
        {0, 1, 0 },
        {0, 1, 1 },
        {1, 0, 0 },
    });
    // T-junc with 1 diag flipped
    // 			#--
    //			-##
    //			-##
    patterns.push_back({
        {1, 0, 0 },
        {0, 1, 1 },
        {0, 1, 1 },
    });
    // T-junc with 1 diag flipped2
    // 			#--
    //			-##
    //			###
    patterns.push_back({
        {1, 0, 0 },
        {0, 1, 1 },
        {1, 1, 1 },
    });

    // 3-way with overlap (1 unset)
    // 			--#
    //			###
    //			-#-
    patterns.push_back({
        {0, 0, 1 },
        {1, 1, 1 },
        {0, 1, 0 },
    });
    // 3-way with overlap
    // 			-##
    //			###
    //			--#
    patterns.push_back({
        {0, 1, 1 },
        {1, 1, 1 },
        {0, 0, 1 },
    });

    // T-jun 2 diag
    // 			-#-
    // 			-#-
    //			#-#
    patterns.push_back({
        {0, 1, 0 },
        {0, 1, 0 },
        {1, 0, 1 },
    });

    // T-jun 2 diag (top set)
    // 			-##
    // 			-#-
    //			#-#
    patterns.push_back({
        {0, 1, 1 },
        {0, 1, 0 },
        {1, 0, 1 },
    });

    // T-jun 2 diag (mid set)
    // 			-#-
    // 			-##
    //			#-#
    patterns.push_back({
        {0, 1, 0 },
        {0, 1, 1 },
        {1, 0, 1 },
    });

    // 2 diag unset
    // 			#-#
    // 			-##
    //			###
    patterns.push_back({
        {1, 0, 1 },
        {0, 1, 1 },
        {1, 1, 1 },
    });
    // 2 diag unset num2
    // 			#-#
    // 			-##
    //			##-
    patterns.push_back({
        {1, 0, 1 },
        {0, 1, 1 },
        {1, 1, 0 },
    });

    // 2 adjacent unset
    //			###
    // 			-##
    // 			-##
    patterns.push_back({
        {1, 1, 1 },
        {0, 1, 1 },
        {0, 1, 1 },
    });

    // 1 unset
    //			###
    // 			##-
    // 			###
    patterns.push_back({
        {1, 1, 1 },
        {1, 1, 0 },
        {1, 1, 1 },
    });

    // T-junc with (bottom set)
    //			-#-
    // 			###
    // 			#--
    patterns.push_back({
        {0, 1, 0 },
        {1, 1, 1 },
        {1, 0, 0 },
    });

    // T-junc with (both bottom set)
    //			-#-
    // 			###
    // 			#-#
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

int makeID(const Pattern& pattern) {
	int id = 0;
	int bit = 0x01;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (pattern[i][j] == GridToGraph::PATH) {
				id |= bit;
			}
			bit <<= 1;
    	}
	}
	return id;
}

// Use the 9 (3x3) bits (bit0 = top left) to created pattern ID
// NOTE: This is called with the Grid and Patterns
//
int getPatternID(const Grid& grid, int x, int y) {
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
	int id = makeID(p);
	// Return true if first time ever stored id
	bool ret = (ids[id] == 0);
	ids[id] = 1;
	return ret;
}

//
// Take the base patterns and rotation them 0,90,180,270 degrees
// also mirror then rotate
// The returned array contains all bit patterbs (using the 9 (3x3) bits)
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

inline bool isNode(int cellValue) {
    // Is NODE or DEND bit set
    return (cellValue & (NODE | DEND)) != 0;
}

inline bool isPath(int cellValue) {
    // It's not empty and not a node
    // (assuming only one bit is set for PATH, or that PATH is bit 1)
    return cellValue && !isNode(cellValue);
}


void debugDump(const Graph& graph);

void extraThining(Grid& grid) {
	// Idea was to change non-thinned
	// ###
	// # #
	// ###
	// by taking corners, but crashed and cant be bothered
#if 0
    Pattern thinPatten = {
        {1, 1, 1 },
        {1, 0, 1 },
        {1, 1, 1 },
    };
    const int extraId = makeID(thinPatten);

    for (int y = 1; y < grid.size() - 3; ++y) {
    	for (int x = 1; x < grid[0].size() - 3; ++x) {
    		const int id = getPatternID(grid, x, y);
    		if (id == extraId) {
    			//if (!grid[y-1][x-1]) grid[y+0][x+0] = EMPTY; // TL
    			//if (!grid[y-1][x+3]) grid[y+0][x+2] = EMPTY; // TR
    			//if (!grid[y+3][x-1]) grid[y+2][x+0] = EMPTY; // BL
    			//if (!grid[y+3][x+3]) grid[y+2][x+2] = EMPTY; // BR
    		}
    	}
    }
#endif
}
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
				std::cerr << "XXXX " << x+1 << "," << y+1 << " SET, ID " << std::hex << id << std::dec << std::endl;
            	if (patternIDS[id]) {
            		// Pattern exists (so it was a match) => store the Node
                	nodes.push_back({x+1, y+1});
					std::cerr << "    XXXX BASENODE: " << x+1 << "," << y+1 << std::endl;
                    
            	}
            }
        }
    }

    return nodes;
}

///////////////////////////////////////////////////////////

namespace {
	void makeTGA2(const char* name, const Grid& grid, unsigned int mask)
	{
#if 1
		unsigned char* pPixel = new unsigned char[grid.size()*grid[0].size()*4];
		unsigned char* pData = pPixel;
		for (int y=grid.size()-1; y>= 0; --y) {
			int x=0;
			for (int xy : grid[y]) {
				xy &= mask;
				switch (xy)
				{
				case GridToGraph::NODE:
					*pData++ = 0xff;
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				case GridToGraph::DEND:
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				case (GridToGraph::XPND<<1):
					*pData++ = 0x00;
					*pData++ = 0xff;
					*pData++ = 0xff;
					*pData++ = 0xff;
					break;
				case (GridToGraph::XPND<<2):
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0xff;
					*pData++ = 0xff;
					break;
				case (GridToGraph::XPND<<3):
					*pData++ = 0x7f;
					*pData++ = 0x00;
					*pData++ = 0xff;
					*pData++ = 0xff;
					break;
				case (GridToGraph::XPND<<2|(XPND<<1)):
					*pData++ = 0x00;
					*pData++ = 0x7f;
					*pData++ = 0xff;
					*pData++ = 0xff;
					break;
				case (GridToGraph::XPND<<3|(XPND<<1)):
					*pData++ = 0x7f;
					*pData++ = 0x00;
					*pData++ = 0x7f;
					*pData++ = 0xff;
					break;
				case (GridToGraph::XPND<<3|(XPND<<2)):
					*pData++ = 0x7f;
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				case (GridToGraph::XPND<<3|(XPND<<2)|(XPND<<1)):
					*pData++ = 0x7f;
					*pData++ = 0x7f;
					*pData++ = 0x7f;
					*pData++ = 0xff;
					break;
				case 0xff:
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0x00;
					*pData++ = 0x00;
					break;
				case GridToGraph::WALL:
					*pData++ = 0x00;
					*pData++ = 0x6f;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				case GridToGraph::XPND:
					*pData++ = 0x4f;
					*pData++ = 0x4f;
					*pData++ = 0x4f;
					*pData++ = 0xff;
					break;
				case GridToGraph::BOUNDARY|GridToGraph::WALL:
					*pData++ = 0x00;
					*pData++ = 0xff;
					*pData++ = 0x00;
					*pData++ = 0xff;
					break;
				default:
					*pData++ = xy ? 0xff : 0x00;
					*pData++ = xy ? 0xff : 0x00;
					*pData++ = xy ? 0xff : 0x00;
					*pData++ = xy ? 0xff : 0x00;
					break;
				}
			}
		}
		bool ret = Stuff::TGA::saveToFile(name, grid[0].size(), grid.size(), 32, pPixel);
		delete[] pPixel;
		std::cerr << "XXXX " << name << " => " << (ret ? "saved" : "*FAILED*") << std::endl;
#endif
	}
	void makeTGA(const char* name, const Grid& grid, bool preProcess=false)
	{
		// Pre-process was just have bottom word (WALL,EMPTY)
		unsigned int mask = preProcess ? 0xffff :  0xffff0000;
		makeTGA2(name, grid, mask);
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
    		if (isPath(grid[p.second][p.first])) {
    			grid[p.second][p.first] = edgeIdx;
    		}
    	}
    	++edgeIdx;
    }
}

// Find all the walls next to a wall and add the BOUNDARY tag
void markGridBoundaries(Grid& grid) {
    int rows = grid.size();
    int cols = grid[0].size();

    for (int r = 1; r < rows-1; ++r) {
        for (int c = 1; c < cols-1; ++c) {
        	// Find non walls
            if (! (grid[r][c]&WALL)) {
            	// Check all the neighbors for a wall
            	for (const auto& [dc, dr] : GridType::directions) {
            		int nc = c + dc;
                    int nr = r + dr;
            		// All walls around a non-wall are boundary
            		if (grid[nr][nc]&WALL) {
            			grid[nr][nc] |= BOUNDARY;  // Mark as BOUNDARY
            		}
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
			if (! isPath(floorGrid[y][x])) {
				infoGrid[y][x] = GridToGraph::WALL;
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
Path simplifyPath(const Path& inPath) {
    Path simplePath;
    if (inPath.size() < 3) return inPath;

    simplePath.push_back(inPath[0]);

    for (size_t i = 1; i < inPath.size() - 1; i++) {
        const auto& prev = inPath[i - 1];
        const auto& curr = inPath[i];
        const auto& next = inPath[i + 1];

        // Check if "prev -> next" is a valid diagonal move
        int dx = next.first - prev.first;
        int dy = next.second - prev.second;
        std::cerr << "SIMPLIFY: " << dx << "" << dy;
        if (std::abs(dx) <= 1 && std::abs(dy) <= 1) {
            // Skip the current point (diagonal move is valid)
            simplePath.push_back(next);
            i++;
			std::cerr << "   => skip " << curr.first<<","<<curr.second<< std::endl;
        }
        else {
            simplePath.push_back(curr);
			std::cerr << "   => keep " << curr.first<<","<<curr.second<< std::endl;
        }
    }

    // Ensure the last point is added
    if (simplePath.back() != inPath.back()) {
        simplePath.push_back(inPath.back());
    }

    return simplePath;
}

/////////////////////////////////////////////////////////////////////////////////////////

// Custom comparator as a lambda function
struct EdgeComparator {
	constexpr bool operator()(const Edge& a, const Edge& b) const {
		if (a.from != b.from) return a.from < b.from;
		if (a.to != b.to) return a.to < b.to;
		if (a.toDeadEnd != b.toDeadEnd) return a.toDeadEnd < b.toDeadEnd;
		return a.path < b.path; // Compare paths
	}
};

class EdgeAdder {
    std::set<Edge, EdgeComparator> edges;

	const Grid& grid;
	std::set<std::pair<int, int>> adjacentNodePairs;

public:
	EdgeAdder(const Grid& g)
	: grid(g)
	{

	}

	// Accessor Methods
    const std::set<Edge, EdgeComparator>& getEdges() { return  edges; }
	const Grid& getGrid() { return grid; }

	// Create a new edge from the Path and add it to the set
	bool addEdge(Path& path) {
		Edge newEdge;
		if (! getEdge(path, newEdge)) {
			return false;
		}

		auto itr = edges.find(newEdge);
		if (itr != edges.end()) {
			std::cerr << "#AddEdge DUP EDGE: " << newEdge.from <<"->"<<newEdge.to << (newEdge.toDeadEnd ? " (DEAD)" : "") << " P: ";
			for (const auto& p : newEdge.path) { std::cerr << p.first <<","<< p.second << "  "; }
			std::cerr << std::endl;
			return false;
		}

		std::cerr << "#AddSEdge STORE: " << newEdge.from <<"->"<< newEdge.to << (newEdge.toDeadEnd ? " (DEAD)" : "") << " P: ";
		for (const auto& p : newEdge.path) { std::cerr << p.first <<","<< p.second << "  "; }
		std::cerr << std::endl;

		edges.insert(itr, newEdge);
		return true;
	}

private:
	// Populate newEdge with the Edge created from the Path (if it is valid)
	bool getEdge(Path& path, Edge& newEdge)
	{
		if (path.empty()) {
			std::cerr << "getEdge => EMPTY" << std::endl;
			return false;
		}
		int sx = path[0].first;
		int sy = path[0].second;
		int ex = path[ path.size()-1 ].first;
		int ey = path[ path.size()-1 ].second;

		int startRC = grid[sy][sx];
		int endRC = grid[ey][ex];

		int node0 = startRC&0xffff;
		int node1 = endRC&0xffff;
		bool isDead0 = (startRC&DEND);
		bool isDead1 = (endRC&DEND);
		bool isDeadEnd = isDead0 || isDead1;
		bool bothDead = isDeadEnd && (isDead0 == isDead1);

		// Should not both be dead and if Node0 == Node1 then one must be a deadEnd
		bool startEndOk = (!bothDead) && ((node0 != node1) || isDeadEnd);
		if (! startEndOk) {
			std::cerr << "#GetEdge START = END: " << (isDead0 ? "DEAD " : "") << node0
					<< " == " << (isDead1 ? "DEAD " : "") << node1 << std::endl;
			return false;
		}

		// 2 nodes next to each other.
		// Store the Pairs and use it to abort early
		if (path.size() == 2) {
			// Store in (smallerNode, largerNode) order
			std::pair<int, int> edge = (node0 < node1)
            						? std::make_pair(node0,node1)
            						: std::make_pair(node1,node0);

			if (adjacentNodePairs.count(edge)) {
				std::cerr << "#GetEdge DUP: " << (isDeadEnd ? "DEAD " : "") << edge.first <<" to "<< edge.second << std::endl;
				return false;
			}
			adjacentNodePairs.insert(edge);
		}

		std::cerr << "#GetEdge " << ((startRC&DEND) ? "DEND " : "NODE ") << (startRC&0xffff)
											<< " -> "
											<< ((endRC&DEND) ? "DEND " : "NODE ") << (endRC&0xffff)
											<< "   P: ";
		for (const auto& p : path) {
			std::cerr << "  " << p.first <<","<< p.second;
		}
		std::cerr << std::endl;

		// Ensure Node1 is the dead, or if none dead then node0 is the lower
		if (isDead0 || (!isDeadEnd && (node0 > node1))) {
			std::swap(isDead0, isDead1);
			std::swap(node0, node1);
			std::reverse(path.begin(), path.end());
		}

		// Simplify the path so wont get duplicates
		Path simplePath = simplifyPath(path);

		if (simplePath.size() != path.size()) {
			std::cerr << "#GetEdge INPUT  ";
			for (const auto& p : path) {
				std::cerr << "  " << p.first <<","<< p.second;
			}
			std::cerr << std::endl;
			std::cerr << "#GetEdge SIMPLE ";
			for (const auto& p : simplePath) {
				std::cerr << "  " << p.first <<","<< p.second;
			}
			std::cerr << std::endl;
		}

		// Make the new Edge to add
		newEdge = {node0, node1, isDeadEnd, simplePath };
		return true;
	}
};


//
// Starting at the given path cell in the grid find the edge and add it
// Visited tracks which cells have been visited for the path except for
// NODE/DEND cells which are never marked as visited (since there are
// multiple paths to them, but path cells are used once
//
Path followPath(int startX, int startY,
                EdgeAdder& edgeAdder,
                std::vector<std::vector<bool>>& visited)
{
    // If we fail to find a valid path, return empty
    Path emptyPath;
    const Grid& grid = edgeAdder.getGrid();

    // Directions in your preferred order: up, left, right, down, then diagonals
    static const std::vector<std::pair<int,int>> directionsHVD = {
        {0, -1},   // up
        {-1, 0},   // left
        {1, 0},    // right
        {0, 1},    // down
        {-1, -1},  // diag up-left
        {1, -1},   // diag up-right
        {-1, 1},   // diag down-left
        {1, 1}     // diag down-right
    };

    // We'll keep a copy of 'visited' to revert if needed
    std::vector<std::vector<bool>> tempVisited = visited;

    // Helper: Mark a PATH cell visited in tempVisited
    auto markPathVisited = [&](int x, int y) {
        tempVisited[y][x] = true;
    };

    // If the start cell is PATH, mark it visited in the temp array
    if ( (grid[startY][startX] != 0)    // not empty
         && ((grid[startY][startX] & (NODE|DEND)) == 0) ) // i.e., it's PATH
    {
        markPathVisited(startX, startY);
    }

    // =========================
    // Try each direction in HVD
    // =========================
    for (auto [dx, dy] : directionsHVD)
    {
        int nx = startX + dx;
        int ny = startY + dy;

        // Skip empty or visited path cells
        if (grid[ny][nx] == 0) continue;   // EMPTY
        if (isPath(grid[ny][nx]) && tempVisited[ny][nx])
        {
        	std::cerr << "  " << nx<<","<<ny<< " => Visited (or Node" << std::endl;
        	continue; // Already visited path cell
        }

        // FORWARD PATH: from neighbor
        std::deque<std::pair<int,int>> forwardPath;
        std::pair<int,int> fwd = {nx, ny};
        std::cerr << "  => FWD Start " << fwd.first <<"," << fwd.second << std::endl;

        while (true)
        {
            // If it's a PATH cell, mark visited
            if (isPath(grid[fwd.second][fwd.first]) && !tempVisited[fwd.second][fwd.first])
            {
                markPathVisited(fwd.first, fwd.second);
                std::cerr << "    VISIT " << fwd.first <<"," << fwd.second << std::endl;
            }

            forwardPath.push_back(fwd);

            // If we reached a node, check if it's the same node or a new one
            if ( isNode(grid[fwd.second][fwd.first]) ) {
                // If it's literally the same node as (startX, startY), skip it
                if (fwd.first == startX && fwd.second == startY) {
                    // This forward direction loops back to the start node => break/fail
                    // We'll try the next direction
                	for (const auto p : forwardPath) {
                		if (isPath(grid[p.second][p.first])) tempVisited[p.second][p.first] = false;
                	}
                    forwardPath.clear();
                    std::cerr << "    LOOP => clear path" << std::endl;
                }
                std::cerr << "  NODE " << fwd.first <<"," << fwd.second << " => BREAK" << std::endl;
                break;
            }

            // Try directions again in HVD order from 'fwd'
            bool foundNext = false;
            for (auto [dx2, dy2] : directionsHVD) {
                int fx = fwd.first + dx2;
                int fy = fwd.second + dy2;

                // Skip empty or visited path
                if (grid[fy][fx] == 0) continue;
                if (isPath(grid[fy][fx]) && tempVisited[fy][fx]) continue;
                std::cerr << "    " << fx << "," << fy << " => Not visited (or is Node) => BREAK" << std::endl;
                // We found an unvisited path or a node => move forward
                fwd = {fx, fy};
                foundNext = true;
                break;
            }
            if (!foundNext) {
                // Could not proceed to any next cell => path fails
                std::cerr << "  NOT FOUND => clear: ";
                for (const auto p : forwardPath) { std::cerr << p.first <<","<< p.second << "  "; }
                std::cerr << std::endl;
                forwardPath.clear();
                break;
            }
        }

        // If forwardPath is empty => that direction failed, try next direction
        if (forwardPath.empty()) {
        	std::cerr << "  EMPTY => Continue" << std::endl;
            continue;
        }

        // BACKWARD PATH: from (startX, startY)
        std::deque<std::pair<int,int>> backwardPath;
        std::pair<int,int> bck = {startX, startY};

        while (true)
        {
        	std::cerr << "  BCK " << bck.first <<","<< bck.second << std::endl;
            // If it's PATH, mark visited
            if (((grid[bck.second][bck.first] & (NODE|DEND)) == 0) // is PATH
                && !tempVisited[bck.second][bck.first])
            {
                markPathVisited(bck.first, bck.second);
                std::cerr << "    VISIT " << bck.first <<"," << bck.second << std::endl;
            }

            backwardPath.push_front(bck);

            // If we reached a node, we stop
            if ((grid[bck.second][bck.first] & (NODE|DEND)) != 0) {
                std::cerr << "    NODE => break" << std::endl;
                break;
            }

            bool foundNext = false;
            for (auto [dx2, dy2] : directionsHVD) {
            	// NOTE: subtract the dir to check dirs in opposite directions to forward
                int bx = bck.first - dx2;
                int by = bck.second - dy2;
                if (grid[by][bx] == 0) continue; // empty
                if (isPath(grid[by][bx]) && tempVisited[by][bx]) continue;

                bck = {bx, by};
                foundNext = true;
                std::cerr << "    " << bx<<","<<by << " => Found next" << std::endl;
                break;
            }
            if (!foundNext) {
                for (const auto p : backwardPath) { std::cerr << p.first <<","<< p.second << "  "; }
                std::cerr << std::endl;
                // Could not proceed => path fails
                backwardPath.clear();
                std::cerr << "    BCK NOT FOUND => clear" << std::endl;
                break;
            }
        }

        // If backward path is empty => that direction fails, revert & try next
        if (backwardPath.empty()) {
        	std::cerr << "  BCK EMPTY => CONTINUE" << std::endl;
            continue;
        }

        // If we get here, we have forward + backward paths => combine them
        // They share the start cell once, but typically that's okay.
		Path result;
        for (auto& p : backwardPath) {
            result.push_back(p);
        }
        for (auto& p : forwardPath) {
            result.push_back(p);
        }

        // We have a valid path => commit changes to 'visited'
        if (edgeAdder.addEdge(result)) {
        	visited = tempVisited;
        	return result;
        }
		tempVisited = visited; // Revert changes
		std::cerr << "  ADD EDGE FAILED => Revert" << std::endl;
        //return emptyPath;
    }

    // If no direction yields a path, revert everything and return empty
    return emptyPath;
}


std::vector<Edge> findEdges(const std::vector<std::vector<int>>& grid,
                            const std::vector<Point>& nodes,
                            const std::vector<Point>& deadEnds)
{
    const int rows = (int)grid.size();
    const int cols = (int)grid[0].size();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

    EdgeAdder edgeAdder(grid);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (grid[y][x] == EMPTY) {
                continue;
            }
            if (isPath(grid[y][x]) && visited[y][x]) {
            	continue;
            }

            std::cerr << std::endl;
            std::cerr <<"#PNT " << x <<","<< y << "   Grid: " << (grid[y][x]) << "\t\t\t" << std::endl;

            Path path = followPath(x,y,edgeAdder,visited);
        }
    }
    const std::set<Edge, EdgeComparator>& edges = edgeAdder.getEdges();
    std::vector<Edge> result;
    result.reserve(edgeAdder.getEdges().size());
    std::copy(edges.begin(), edges.end(), std::back_inserter(result));
    return result;
}

std::vector<Edge> findNodeEdges(const std::vector<Point>& nodes, const std::vector<std::vector<int>>& grid)
{
    const int rows = (int)grid.size();
    const int cols = (int)grid[0].size();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

    EdgeAdder edgeAdder(grid);

    std::cerr << "## FIND NODE EDGES for " << nodes.size() << " unconnected nodes" << std::endl;
    for (const auto& n : nodes)
    {
        int x = n.first;
        int y = n.second;
        Path path;
        do {
            path = followPath(x, y, edgeAdder, visited);
        } while (! path.empty());
    }
    const std::set<Edge, EdgeComparator>& edges = edgeAdder.getEdges();
    std::vector<Edge> result;
    result.reserve(edgeAdder.getEdges().size());
    std::copy(edges.begin(), edges.end(), std::back_inserter(result));
    std::cerr << "  FOUND " << result.size() << " edges" << std::endl;
    return result;
}


BaseGraph fixBaseEdges(std::vector<Edge>& baseEdges, const Grid& infoGrid, const std::vector<Point>& baseNodes)
{
	// First Make a graph edgeNode1 -> list of (edgeNode2, edgeIndex, 1->2 flag, cost)
    BaseGraph baseGraph = GridType::buildBaseGraph(baseEdges, baseNodes.size());

	// Use graph to check all base nodes connected
	std::vector<int> unconnected = GridType::checkConnectivity(baseGraph, baseNodes.size());
    if (unconnected.empty())
    {
        return baseGraph;
    }

    // Turn list of unconnected node indexes to list of Nodes
	std::vector<Point> unconnectedNodes;
	for (int idx : unconnected) {
		unconnectedNodes.push_back(baseNodes[idx]);
	}

    // Find the edges for those nodes
	std::vector<Edge> connections = findNodeEdges(unconnectedNodes, infoGrid);

    // Make an EdgeComparator set of all current edges
    GridToGraph::EdgeComparator comp;
    std::set<Edge, EdgeComparator> edgeSet(baseEdges.begin(), baseEdges.end(), comp);
	for (const auto& e : connections)
	{
        if (edgeSet.find(e) == edgeSet.end())
        {
            edgeSet.insert(e);
            baseEdges.push_back(e);
        }
	}

	// Return the new graph with the new edges
    return buildBaseGraph(baseEdges, baseNodes.size());
}

/////////////////////////////////////////////////////////////////////////////////////////

void expandPaths(Grid& grid)
{
    int rows = grid.size();
    int cols = grid[0].size();

    std::queue<std::pair<Point, int>> queue;  // {position, distance}

    // Initialize queue with all EDGE points from infoGrid
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
			// Only EDGE points start BFS
            if (grid[r][c] & GridToGraph::EDGE) {
                queue.push({{c, r}, 0});
            }
        }
    }

    // Process BFS queue
    while (!queue.empty()) {
        auto [pos, dist] = queue.front();
        queue.pop();

        int c = pos.first;
    	int r = pos.second;

        // Attempt to expand in all 8 directions
        for (const auto& [dc, dr] : directions) {
            int nc = c + dc;
            int nr = r + dr;

            // Check bounds and ensure we only expand empty cells
            if (grid[nr][nc] == GridToGraph::EMPTY) {
                int newDist = dist + 1;

                grid[nr][nc] = (newDist & 0xFFFF) | GridToGraph::XPND;
                queue.push({{nc, nr}, newDist});
            }
        }
    }
}


// Euclidean distance function
double euclideanDistance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.first - b.first, 2) + std::pow(a.second - b.second, 2));
}

// Finds the base node closest to the cluster center
int findCentralNode(const AbstractNode& abstractNode, const std::vector<Point>& nodes) {
	std::cerr << "FIND CENTRAL NODE: " << abstractNode.center.first << "," << abstractNode.center.second << " " << abstractNode.baseNodes.size() << std::endl;
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

// **Finds the shortest path between two base nodes using BFS**
Path findPathBetweenNodes(int startNode, int endNode, const std::vector<Edge>& edges, const std::vector<Point>& nodes) {
    std::queue<int> q;
    std::vector<int> prev(nodes.size(), -1);
    std::vector<int> edgeUsed(nodes.size(), -1);
    std::vector<bool> visited(nodes.size(), false);

    std::cerr << "PATH BETWEEN: " << startNode << " -> " << endNode << std::endl;
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
            	std::cerr << "  Checked edge: " << edgeIdx << " " << edge.from << " -> " << edge.to
            			<< " cur: " << current << " neigh: " << neighbor << std::endl;
                visited[neighbor] = true;
                prev[neighbor] = current;
                edgeUsed[neighbor] = edgeIdx;
                q.push(neighbor);
                std::cerr << "  store: " << neighbor << " = " << current << std::endl;
            }
        }
    }

    // If no path was found, return empty path
    if (prev[endNode] == -1) return {};

    // **Reconstruct the path from startNode → endNode**
    Path fullPath;
    int at = endNode;

    while (at != startNode) {
        int edgeIdx = edgeUsed[at];
        if (edgeIdx == -1) break;

        const auto& edgePath = edges[edgeIdx].path;
        if (edges[edgeIdx].to == at) {
            fullPath.insert(fullPath.end(), edgePath.begin(), edgePath.end());
        } else {
            fullPath.insert(fullPath.end(), edgePath.rbegin(), edgePath.rend());
        }

        at = prev[at];
    }
    std::cerr << "  NODES: ";
    for (int n : prev) {
    	std::cerr << n << " ";
    }
    std::cerr << std::endl;
    std::cerr << "FINAL PATH BETWEEN: " << startNode << " -> " << endNode << std::endl;
    for (const auto& pnt : fullPath) {
    	std::cerr << "  " << pnt.first << "," << pnt.second << std::endl;
    }

    return fullPath;
}

static std::vector<int> dbscan(const std::vector<Point>& points, double eps, int minPts)
{
    const int UNVISITED = -1;
    const int NOISE = -2;

    std::vector<int> labels(points.size(), UNVISITED);
    int clusterId = 0;

    auto regionQuery = [&](const Point& p) -> std::vector<int> {
        std::vector<int> neighbors;
        for (size_t i = 0; i < points.size(); ++i) {
            if (euclideanDistance(p, points[i]) <= eps) {
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
////////////////////////////////////////////////////////////////////

// **Creates the Abstract Graph from clustered base nodes**
std::vector<AbstractNode> createAbstractNodes(
    const std::vector<Point>& nodes,
    const std::vector<Edge>& edges,
    double clusteringEps,
    int minClusterSize)
{
    // Step 1: Cluster points
    std::vector<int> clusterLabels = dbscan(nodes, clusteringEps, minClusterSize);
    for (int i=0; i < clusterLabels.size(); ++i) {
    	std::cerr << "  CLUSTER " << i << " => " << clusterLabels[i]<< std::endl;
    }
    // Step 2: Create Abstract Nodes
    std::unordered_map<int, AbstractNode> clusters;
    for (size_t i = 0; i < nodes.size(); ++i) {
        int clusterId = clusterLabels[i];
		// Skip noise points (outliers)
        if (clusterId < 0) {
			continue;   
        }

        if (clusters.find(clusterId) == clusters.end()) {
            clusters[clusterId] = {{}, {0, 0}, -1};
        }

        clusters[clusterId].baseNodes.push_back(static_cast<int>(i));
        clusters[clusterId].center.first += nodes[i].first;
        clusters[clusterId].center.second += nodes[i].second;
    }

	// Calculate the center of each cluster
	std::cerr << "CLUSTERS: " << clusters.size() << std::endl;
    for (auto& [i, cluster] : clusters) {
        int totalPoints = cluster.baseNodes.size();
        cluster.center.first /= totalPoints;
        cluster.center.second /= totalPoints;
    }
    // Step 3: Collect abstract nodes into a vector
    std::vector<AbstractNode> abstractNodes;
    for (const auto& [_, cluster] : clusters) {
    	abstractNodes.push_back(cluster);
    }

    // Step 4: Assign closest base node to each abstract node
    std::unordered_map<int, int> closestMap;
    int idx=0;
    for (auto& abNode : abstractNodes) {
        int baseIdx = findCentralNode(abNode, nodes);
		if (baseIdx == -1) {
			std::cerr << "ERROR: Could not find central node for idx: " << idx << std::endl;
		}
        abNode.baseCenterNode = baseIdx;
        closestMap[idx] = baseIdx;
        const auto& f = nodes[ abNode.baseCenterNode ];
        std::cerr << "MADE ABNODE " << idx << ": centerBase:" << abNode.baseCenterNode << " ("<<f.first<<","<<f.second<<")" << std::endl;
        ++idx;
    }

    return abstractNodes;
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
                for (const auto& [dc, dr] : directions) {
                	int nc = c + dc;
                    int nr = r + dr;

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

        for (const auto& [dc, dr] : directions) {
        	int nc = c + dc;
            int nr = r + dr;

            if (fallbackGrid[nr][nc].distance != -1 || flowFieldCoverage[nr][nc] != 0) {
                continue;  // Skip walls & already processed points
            }

            // Assign new fallback cell
            const FallbackCell& cell =fallbackGrid[r][c];
            fallbackGrid[nr][nc] = {targetX, targetY, cell.distance+1};
            queue.emplace(nr, nc, targetX, targetY);  // Expand further
        }
    }
	return fallbackGrid;
}

////////////////////////////////////////////////////////

//
// Generates the Voronoi-like zones around each AbstractNode using path-based BFS
// - Populates NavGrid with closest AbstractNode index and distance to it
//
std::vector<ZoneInfo> generateAbstractZones(NavGrid& navGrid,
    const Grid& grid,
    const BaseGraph& baseGraph,
	const std::vector<Edge>& baseEdges,
    const std::vector<AbstractNode>& abstractNodes)
{
    int rows = grid.size();
    int cols = grid[0].size();

    // Create NavGrid. Note that GridPointInfo inis each field to appropriate value
	navGrid = NavGrid(rows, std::vector<GridPointInfo>(cols));

    using QueueElement = std::tuple<int, int, int, double>; // (row, col, abstractNodeIdx, pathCost)
    std::queue<QueueElement> q;

    // Initialize BFS from each Abstract Node
    for (size_t i = 0; i < abstractNodes.size(); ++i) {
        Point nodePos = abstractNodes[i].center;
		// Add abstract node to queue at dist 0
        q.push({ nodePos.first, nodePos.second, static_cast<int>(i), 0 });
		// set the navgrid closes node to itself and dist to 0
        navGrid[nodePos.second][nodePos.first].closestAbstractNodeIdx = i;
        navGrid[nodePos.second][nodePos.first].distanceToAbstractNode = 0;
    }

    // Multi-source path-based BFS
    while (!q.empty()) {
        auto [c, r, abstractIdx, cost] = q.front();
        q.pop();

		// Check the neighbors of the cur zone cell
        for (const auto& [dc, dr] : directions) {
            int nc = c + dc;
            int nr = r + dr;

            if (grid[nr][nc]&WALL) continue;

            // Expand cost of zone
            int newCost = cost + 1;

            // Only update if we found a shorter path (dist stats at MAX)
            if (newCost < navGrid[nr][nc].distanceToAbstractNode) {
                navGrid[nr][nc].closestAbstractNodeIdx = abstractIdx;
                navGrid[nr][nc].distanceToAbstractNode = newCost;
				// Add this cell to the queue for further expansion
                q.emplace(nc, nr, abstractIdx, newCost);
            }

        }
    }
    std::vector<ZoneInfo> zones(abstractNodes.size());

    // If this edge crosses zones update the zone infos
	auto addEdge = [baseEdges, navGrid](int edgeIdx, int curZone, std::vector<ZoneInfo>& zones) -> void {
		Point from = baseEdges[edgeIdx].path.front();
		Point to = baseEdges[edgeIdx].path.back();
		int fromZone = navGrid[from.second][from.first].closestAbstractNodeIdx;
		int toZone = navGrid[to.second][to.first].closestAbstractNodeIdx;
        // If the edge crosses zones
		if (fromZone != toZone) {
			// The boundaryZone is the zone that is not the current zone
			int boundaryZone = (fromZone == curZone) ? toZone : fromZone;
			// Add the boundaryZone if not already in the adjacentZones list
			if (std::find(zones[curZone].adjacentZones.begin(), zones[curZone].adjacentZones.end(), boundaryZone) == zones[curZone].adjacentZones.end()) {
				zones[curZone].adjacentZones.push_back(boundaryZone);
				zones[boundaryZone].adjacentZones.push_back(curZone);
			}
		}
		// Add the edge to the lit of edges for the the current zone
		if (std::find(zones[curZone].baseEdgeIdxs.begin(), zones[curZone].baseEdgeIdxs.end(), edgeIdx) == zones[curZone].baseEdgeIdxs.end()) {
			zones[curZone].baseEdgeIdxs.push_back(edgeIdx);
		}
	};

    // Skip boundary cells
	for (int y = 1; y < rows-1; ++y) {
		for (int x = 1; x < cols-1; ++x) {
			int cell = grid[y][x];
			if (cell & WALL) continue;

            int curZone = navGrid[y][x].closestAbstractNodeIdx;
            int curBaseNode = abstractNodes[curZone].baseCenterNode;

			// Bottom bits of infoGrid are node or edge index
			int idx = cell & 0xFFFF;

			// If node then add to zone's list
			if (cell & NODE) {
				zones[curZone].baseNodes.push_back(idx);
				// Corner case of 2 nodes next to each other so no EDGE cell to check
				for (const auto& [dx, dy] : directions) {
					int nx = x + dx;
					int ny = y + dy;
					int dirCell = grid[ny][nx];
					if (dirCell & NODE) {
						// index of the adjacent node
						int otherIdx = dirCell & 0xFFFF;
                        // Performance: Only check for shortEdge when at the smaller index node
                        // Since nodes are adjacent and we will hit both nodes since checking every cell
                        if (idx < otherIdx) {
                            // Get the BaseGraphInfo for the other node
                            const auto& edges = baseGraph.at(idx);
                            for (const auto& bi : edges) {
                                if (bi.neighbor == otherIdx) {
                                    std::cerr << "## ADD EDGE: " << x << "," << y << " zone:" << curZone << " shortEdge: " << bi.edgeIndex << " (" << idx << "<->" << otherIdx << ")" << std::endl;
                                    addEdge(bi.edgeIndex, curZone, zones);
                                    break;
                                }
                            }
                        }
					}
				}
            }
            // If it's an EDGE then add it
			else if (cell & EDGE) {
				if (baseEdges[idx].toDeadEnd) continue;
                std::cerr << "## ADD EDGE: " << x << "," << y << " zone:" << curZone << " edge:" << idx << std::endl;
				addEdge(idx, curZone, zones);
           }
		}
	}

    return zones;
}

////////////////////////////////////////////////////////

std::unordered_map<int, std::unordered_set<int>> detectZoneBoundaries(const NavGrid& zoneGrid)
{
	std::cerr << "DETECT ADJ ZONES" << std::endl;
	std::unordered_map<int, std::unordered_set<int>> adjacentZones;

    int rows = zoneGrid.size();
    int cols = zoneGrid[0].size();

    for (int y = 1; y < rows-1; ++y) {
        for (int x = 1; x < cols-1; ++x) {
            int zone = zoneGrid[y][x].closestAbstractNodeIdx;
            if (zone == -1) continue;  // Ignore walls

            // Check 4 neighbors
			for (const auto& [dx, dy] : std::vector<std::pair<int, int>>{ {0,1}, {0,-1}, {1,0}, {-1,0} }) {
				int nx = x + dx;
				int ny = y + dy;
				int neighborZone = zoneGrid[ny][nx].closestAbstractNodeIdx;
				if (neighborZone == -1) continue;  // Ignore walls

				// Add to adjacency list
				if (neighborZone != zone) {
					std::cerr << "BOUNDARY: " << x << "," << y << " zone:" << zone << " neigh:" << neighborZone << std::endl;   
					adjacentZones[zone].insert(neighborZone);
					adjacentZones[neighborZone].insert(zone);
				}
			}
        }
    }
	return adjacentZones;
}

bool areZonesAdjacent(const Graph& graph, const Point& a, const Point& b) {
    int zoneA = graph.navGrid[a.second][a.first].closestAbstractNodeIdx;
    int zoneB = graph.navGrid[a.second][a.first].closestAbstractNodeIdx;
    return graph.adjacentZones.at(zoneA).count(zoneB) > 0;
}


////////////////////////////////////////////////////////
	

// Function to compute the shortest path using BFS (fallback when no direct BaseEdge exists)
std::vector<std::pair<int, int>> computeShortestPath(
    const std::pair<int, int>& start,
    const std::pair<int, int>& end,
    const NavGrid& zoneGrid)
{
    std::queue<std::vector<std::pair<int, int>>> q;
    std::unordered_set<std::pair<int, int>, PairHash> visited;

    q.push({ start });
    visited.insert(start);

    while (!q.empty()) {
        std::vector<std::pair<int, int>> path = q.front();
        q.pop();

        std::pair<int, int> current = path.back();
        if (current == end) {
            return path; // Shortest path found
        }

        for (const auto& [dx, dy] : directions) {
            std::pair<int, int> next = { current.first + dx, current.second + dy };

            if (zoneGrid[next.second][next.first].closestAbstractNodeIdx == -1 || visited.count(next)) {
                continue;
            }

            visited.insert(next);
            std::vector<std::pair<int, int>> newPath = path;
            newPath.push_back(next);
            q.push(newPath);
        }
    }

    return {}; // No path found (shouldn't happen in a valid map)
}

// Main function to generate AbstractEdges
void makeExtraAbstractEdges(const Graph& graph, std::vector<AbstractEdge>& abstractEdges)
{
	std::cerr << "## MAKE EXTRA ABSTRACT EDGES" << std::endl;
    // make current connections
	std::unordered_map<std::pair<int, int>, bool, PairHash> connectedZones;
	for (const auto& edge : graph.abstractEdges) {
		connectedZones[{edge.from, edge.to}] = true;
		connectedZones[{edge.to, edge.from}] = true;
	}

	// For each zone, check it's adjacent zones and add edges if not already connected
    std::vector<AbstractEdge> extraEdges;
	for (int zoneA = 0; zoneA < graph.abstractNodes.size(); ++zoneA) {
		const ZoneInfo& infoA = graph.zones[zoneA];
        for (int zoneB : infoA.adjacentZones) {
			// Check already connected
            if (connectedZones.find({ static_cast<int>(zoneA), zoneB }) != connectedZones.end()) continue;

            // Connect the two zones
            connectedZones[{zoneA, zoneB}] = true;
            connectedZones[{zoneB, zoneA}] = true;


            std::vector<AbstractNode> adjacentNodes;
            adjacentNodes.push_back(graph.abstractNodes[zoneA]);
            adjacentNodes.push_back(graph.abstractNodes[zoneB]);
			std::vector<Edge> adjacentBaseEdges;
			bool isPossible = false;
            // check all the edges in the zone
            for (int baseIdx : infoA.baseEdgeIdxs) {
				// For this edge get the from and to zones that they are closest to
                Edge adjacentEdge = graph.baseEdges[baseIdx];
                int toZone = graph.navGrid[adjacentEdge.path.back().second][adjacentEdge.path.back().first].closestAbstractNodeIdx;
                int fromZone = graph.navGrid[adjacentEdge.path.front().second][adjacentEdge.path.front().first].closestAbstractNodeIdx;
                std::cerr << "    baseIdx:" << baseIdx << " "  << adjacentEdge.from << "->" << adjacentEdge.to  << " p:" << adjacentEdge.path.size()
                    << " front: "
                    << adjacentEdge.path.front().first << "," << adjacentEdge.path.front().second
                    << " back: "
                    << adjacentEdge.path.back().first << "," << adjacentEdge.path.back().second
            		<< " fromZ: " << fromZone << " toZ: " << toZone << std::endl;
                // If both ends of the edge area closest to the current zone then just add edge to the list
                if (toZone == fromZone)
                {
                    adjacentBaseEdges.push_back(adjacentEdge);
                    std::cerr << "    Use internal zoneA edge " << adjacentEdge.from << "->" << adjacentEdge.to << std::endl;
                }
				// If one of the edges is closest to the other zone then add it to the list and remember we can get to other zone
                else if ((toZone == zoneB) || (fromZone == zoneB)) {
                    adjacentBaseEdges.push_back(adjacentEdge);
                    std::cerr << "    Connector edge " << adjacentEdge.from << "->" << adjacentEdge.to << std::endl;
                    isPossible = true;
                }
            }
			// If none of the edges connect to the other zone then skip
            if (!isPossible) continue;

            // We have at least one edge that connects the zones add all the internal edges of other zone to list
			const ZoneInfo& infoB = graph.zones[zoneB];
            for (int baseIdx : infoB.baseEdgeIdxs) {
                Edge adjacentEdge = graph.baseEdges[baseIdx];
                int toZone = graph.navGrid[adjacentEdge.path.back().second][adjacentEdge.path.back().first].closestAbstractNodeIdx;
                int fromZone = graph.navGrid[adjacentEdge.path.front().second][adjacentEdge.path.front().first].closestAbstractNodeIdx;
				// An internal edge has both ends closest to the other zone
                if (toZone == fromZone)
                {
                    adjacentBaseEdges.push_back(adjacentEdge);
					std::cerr << "    Use internal zoneB edge " << adjacentEdge.from <<"->" << adjacentEdge.to << std::endl;
                }
            }

			std::cerr << "###### ZONE: a:" << zoneA << " b: " << zoneB << " try find route: edges: " << adjacentBaseEdges.size() << std::endl;
			std::vector<AbstractEdge> newEdges;
			AbstractMST::generateMSTAbstractEdges(graph.baseGraph, graph.baseEdges, graph.baseNodes, adjacentNodes, newEdges);
            if (newEdges.size() > 1)
            {
				std::cerr << "************** EEEEK ********** " << std::endl;
            }
            std::cerr << "###### => GOT " << newEdges.size() << " extra edges" << std::endl;
			for (const auto& newEdge : newEdges) {
                const Edge& fromBase = adjacentBaseEdges[newEdge.from];
                const Edge& toBase = adjacentBaseEdges[newEdge.to];
				std::cerr << " ***** ADDed zone connect: " << zoneA<<"->"<<zoneB << "  EDGE:   frombase: " << fromBase.from << "->" << fromBase.to << "  toBase: " << toBase.from<<"->" << toBase.to << std::endl;
				std::cerr << "       Path: " << newEdge.path.size() << std::endl;
                for (const auto& p : newEdge.path) {
                    std::cerr << p.first << "," << p.second << "  ";
                }
				std::cerr << std::endl;
				extraEdges.push_back({ zoneA, zoneB, newEdge.path });
			}
        }
	}
	std::cerr << "EXTRA EDGES: " << extraEdges.size() << std::endl;
    abstractEdges.insert(abstractEdges.end(), extraEdges.begin(), extraEdges.end());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

Graph makeGraph(const Grid& floorGrid)
{
    //
    // Need to copy the floorGrid since ZSThinning removes floor
    //
    Graph graph;
    graph.infoGrid = floorGrid;
    {
        auto tempGrid = graph.infoGrid;

        for (auto& r : tempGrid) {
            for (auto& c : r) {
                c = (c == EMPTY) ? WALL : EMPTY;
            }
        }
        makeTGA("GRID_INPUT.tga", tempGrid);
    }

    //
    // Thin the floor down to a single path
    //
    Algo::ZSThinning(graph.infoGrid);
    extraThining(graph.infoGrid);
    makeTGA("GRID_THIN.tga", graph.infoGrid, true);

    //
    // Get the deadEnds and Nodes
    //
    graph.deadEnds = detectDeadEnds(graph.infoGrid);
    graph.baseNodes = detectNodes(graph.infoGrid);
    {
        Grid tempGrid(graph.infoGrid.size(), std::vector<int>(graph.infoGrid[0].size(), EMPTY));
        for (int n = 0; n < graph.baseNodes.size(); ++n) {
            std::cerr << "## BASE NODE: base: " << n << " (";
            int c = graph.baseNodes[n].first;
            int r = graph.baseNodes[n].second;
            std::cerr << c << "," << r << ")" << std::endl;
            tempGrid[r][c] = NODE;
        }
        makeTGA("GRID_NODES.tga", tempGrid);
    }

    //
    // Set NODE and DEND in infoGrid
    //
    markGridNodes(graph.infoGrid, graph.baseNodes, graph.deadEnds);

    //
    // Find all the BaseEdges connecting all the NODE and DENDs
    //
	graph.baseEdges = findEdges(graph.infoGrid, graph.baseNodes, graph.deadEnds);

    //
    // Unfortunately there are a few scenarios that dont connect. Basically
	// some places aren't marked as nodes (some areas where "double/triple thick"
	// This allows a path between A and B that bypasses C, but marks the
	// path cells as visited, so C doesnt have a path of A or B e.g.
	//
	//    A--x---B    x isn't marked as a node
	//       |        (real scenario is more messy)
    //       C        So A->B cells visited => no paths from C
    //
    // To fix it have version of findEdges that just checks for paths from the
    // list of nodes. Any new edges are added to the complete list
	// 
    graph.baseGraph = fixBaseEdges(graph.baseEdges, graph.infoGrid, graph.baseNodes);

    //
    // Mark the edges in the infoGrid and their index
    //
	markGridEdges(graph.infoGrid, graph.baseEdges);
    {
        std::vector<int> uncon = checkConnectivity(graph.baseGraph, graph.baseNodes.size());
		Grid tempGrid(graph.infoGrid.size(), std::vector<int>(graph.infoGrid[0].size(), EMPTY));
        for (const auto& e : graph.baseEdges) {
            for (int i = 1; i < e.path.size() - 1; ++i) {
                int c = e.path[i].first;
                int r = e.path[i].second;
                tempGrid[r][c] = PATH;
            }
			std::cerr << std::endl;
            int c = e.path[0].first;
            int r = e.path[0].second;
            tempGrid[r][c] = (std::find(uncon.begin(), uncon.end(), e.from) == uncon.end()) ? NODE : XPND << 2;
			c = e.path[e.path.size()-1].first;
			r = e.path[e.path.size()-1].second;
            tempGrid[r][c] = (e.toDeadEnd) ? DEND : (std::find(uncon.begin(), uncon.end(), e.from) == uncon.end()) ? NODE : XPND << 2;
		}
        makeTGA2("GRID_EDGES.tga", tempGrid, 0xffffffff);
	}

	//
	// - Restore Walls (since thinning make FLOOR = EMPTY, then expand Paths to fill EMPTY
    // - Mark the WALLs which are/ next to an empty cell with BOUNDARY
    // - Expand out the paths to mark them as XPND
	//
	restoreWalls(graph.infoGrid, floorGrid);
	markGridBoundaries(graph.infoGrid);
	expandPaths(graph.infoGrid);

    //
    // Find the abstract Nodes
	//
	graph.abstractNodes = createAbstractNodes(graph.baseNodes, graph.baseEdges, 5.0, 2);
	{
		Grid tempGrid(graph.infoGrid.size(), std::vector<int>(graph.infoGrid[0].size(), EMPTY));
        for (const auto& e : graph.baseEdges) {
            std::cerr << "## BASEEDGE " << e.from << " -> " << e.to << " (dead: " << e.toDeadEnd << ") " << " path: " << e.path.size() << std::endl;
            for (int i = 1; i < e.path.size() - 1; ++i) {
                int c = e.path[i].first;
                int r = e.path[i].second;
                tempGrid[r][c] = XPND << 1;
            }
            if (e.path.size() > 1) {
                int c = e.path[0].first;
                int r = e.path[0].second;
                tempGrid[r][c] = NODE;
                c = e.path[e.path.size() - 1].first;
                r = e.path[e.path.size() - 1].second;
                tempGrid[r][c] = NODE;
            }
		}
        for (int n = 0; n < graph.baseNodes.size(); ++n) {
            std::cerr << "## BASE NODE: base: " << n << " (";
			int c = graph.baseNodes[n].first;
			int r = graph.baseNodes[n].second;
            std::cerr << c << "," << r << ")" << std::endl;
			tempGrid[r][c] = NODE;
		}
		for (const auto& n : graph.abstractNodes) {
            std::cerr << "## ABSTRACT NODE: base: " << n.baseCenterNode << " (";
			int c = graph.baseNodes [n.baseCenterNode ].first;
			int r = graph.baseNodes [n.baseCenterNode ].second;
            std::cerr << c << "," << r << ")" << std::endl;
			tempGrid[r][c] = XPND<<2;
		}
		makeTGA("GRID_ALL_NODES.tga", tempGrid);
	}

    //
    // Make Min Span Tree of abstract nodes to get abstract edges
    //
	AbstractMST::generateMSTAbstractEdges(graph.baseGraph, graph.baseEdges, graph.baseNodes, graph.abstractNodes, graph.abstractEdges);
	{
		Grid tempGrid(graph.infoGrid.size(), std::vector<int>(graph.infoGrid[0].size(), EMPTY));
		int idx=0;
		for (; idx < graph.abstractEdges.size(); ++idx) {
			const auto& e = graph.abstractEdges.at(idx);
			for (int i=1; i < e.path.size()-1; ++i) {
				int c = e.path[i].first;
				int r = e.path[i].second;
                if (tempGrid[r][c] != (XPND<<2)) tempGrid[r][c] = XPND<<1;
			}
			int c = e.path[e.path.size()-1].first;
			int r = e.path[e.path.size()-1].second;
			tempGrid[r][c] = XPND<<2;
			c = e.path[0].first;
			r = e.path[0].second;
			tempGrid[r][c] = XPND<<2;
			std::cerr << "## ABEDGE "<<idx<<"   ab:" << e.from << " -> " << e.to << " (base: "
					<< graph.abstractNodes[ e.from ].baseCenterNode << " -> " << graph.abstractNodes[ e.to ].baseCenterNode << ")" << std::endl;
		}
		std::cerr << "BASENODE: " << graph.baseNodes.size()<< std::endl;
		std::cerr << "BASEEDGE: " << graph.baseEdges.size()<< std::endl;
		std::cerr << "ABNODE: " << graph.abstractNodes.size()<< std::endl;
		std::cerr << "ABEDGE: " << graph.abstractEdges.size()<< std::endl;
		makeTGA("GRID_ABSTRACT.tga", tempGrid);
	}

    //
    // Generate a zone for each abstract node
    //
	graph.zones = generateAbstractZones(graph.navGrid, graph.infoGrid, graph.baseGraph, graph.baseEdges, graph.abstractNodes);
	{
		std::vector<int> vals = {
				GridToGraph::NODE,
				GridToGraph::DEND,
				(GridToGraph::XPND << 1),
				(GridToGraph::XPND << 2),
				(GridToGraph::XPND << 3),
				(GridToGraph::XPND<<2)|(XPND<<1),
				(GridToGraph::XPND<<3)|(XPND<<1),
				(GridToGraph::XPND<<3)|(XPND<<2),
				(GridToGraph::XPND<<3)|(XPND<<2)|(XPND<<1),
				GridToGraph::WALL,
				GridToGraph::XPND,
				GridToGraph::BOUNDARY | GridToGraph::WALL,
				};

        int cols = graph.infoGrid[0].size();
        int rows = graph.infoGrid.size();
		Grid tempGrid(rows, std::vector<int>(cols, EMPTY));
		for (int row = 0; row < graph.infoGrid.size(); ++row) {
            for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
                int zn = graph.navGrid[row][col].closestAbstractNodeIdx;
                if (zn != -1) {
                    int val = vals[zn % vals.size()];
                    tempGrid[row][col] = val;
                }
            }
		}
		makeTGA("GRID_ZONES.tga", tempGrid);
	}
    graph.adjacentZones = detectZoneBoundaries(graph.navGrid);
	{
		for (const auto& [zone, neighbors] : graph.adjacentZones) {
			std::cerr << "ZONE " << zone << " => ";
			for (const auto& n : neighbors) {
				std::cerr << n << " ";
			}
			std::cerr << std::endl;
		}
        int i = 0;
		for (const auto& zi : graph.zones) {
			std::cerr << "INFO " << i << " => ";
			for (const auto& n : zi.adjacentZones) {
				std::cerr << n << " ";
			}
			std::cerr << "         Nodes: ";
			for (const auto& n : zi.baseNodes) {
				std::cerr << n << " ";
			}
			std::cerr << "         Edges: ";
			for (const auto& n : zi.baseEdgeIdxs) {
				std::cerr << n << " ";
			}
            ++i;
			std::cerr << std::endl;
		}
	}
    makeExtraAbstractEdges(graph, graph.abstractEdges);
	{
		Grid tempGrid(graph.infoGrid.size(), std::vector<int>(graph.infoGrid[0].size(), EMPTY));
		int idx=0;
		for (; idx < graph.abstractEdges.size(); ++idx) {
			const auto& e = graph.abstractEdges.at(idx);
			for (int i=1; i < e.path.size()-1; ++i) {
				int c = e.path[i].first;
				int r = e.path[i].second;
                if (tempGrid[r][c] != (XPND<<2)) tempGrid[r][c] = XPND<<1;
			}
			int c = e.path[e.path.size()-1].first;
			int r = e.path[e.path.size()-1].second;
			tempGrid[r][c] = XPND<<2;
			c = e.path[0].first;
			r = e.path[0].second;
			tempGrid[r][c] = XPND<<2;
			std::cerr << "## ABEDGE "<<idx<<"   ab:" << e.from << " -> " << e.to << " (base: "
					<< graph.abstractNodes[ e.from ].baseCenterNode << " -> " << graph.abstractNodes[ e.to ].baseCenterNode << ")" << std::endl;
		}
		std::cerr << "BASENODE: " << graph.baseNodes.size()<< std::endl;
		std::cerr << "BASEEDGE: " << graph.baseEdges.size()<< std::endl;
		std::cerr << "ABNODE: " << graph.abstractNodes.size()<< std::endl;
		std::cerr << "ABEDGE: " << graph.abstractEdges.size()<< std::endl;
		makeTGA("GRID_ABSTRACT2.tga", tempGrid);
	}


	//
	// Generate sparse flow fields for all the abstract edges
	// and the fallback graph to use when no flow field
	//
    FlowField::generateFlowFieldsParallel(&graph, graph.flowFields);


	{
		std::vector<int> vals = {
				GridToGraph::NODE,
				GridToGraph::DEND,
				(GridToGraph::XPND << 1),
				(GridToGraph::XPND << 2),
				GridToGraph::WALL,
				GridToGraph::XPND,
				GridToGraph::BOUNDARY | GridToGraph::WALL,
				};

        int cols = graph.infoGrid[0].size();
        int rows = graph.infoGrid.size();
        int ffNum = -1;
		for (const auto ff : graph.flowFields) {
            ++ffNum;
            int val = vals[ffNum % vals.size()];
			Grid tempGrid(rows, std::vector<int>(cols, EMPTY));
			for (int row=0; row < graph.infoGrid.size(); ++row) {
                for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
                    const auto& dirNcost = ff.unpack({ col, row });
                    if (dirNcost) {
                        tempGrid[row][col] = val;
                    }
                    else {
                        tempGrid[row][col] = EMPTY;
                    }
                }
			}
            std::string fnam = "GRID_FF_" + std::to_string(ffNum) + ".tga";
			makeTGA(fnam.c_str(), tempGrid);
		}
	}


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
	int idx=0;
	for (const auto& node : graph.abstractNodes) {
		std::cerr << " " << idx << " Center: (" << node.center.first << ", " << node.center.second << ")"
				<< " nodesSz: " << node.baseNodes.size()
				<< " baseIdx:" << node.baseCenterNode << " ("
				<< graph.baseNodes[node.baseCenterNode].first << "," << graph.baseNodes[node.baseCenterNode].second << ")"
				<< std::endl;
		++idx;
	}

	std::cerr << "\nAbstract Edges: " << graph.abstractEdges.size() << std::endl;
	for (const auto& edge : graph.abstractEdges) {
		std::cerr << "Edge from Cluster " << edge.from << " to Cluster " << edge.to << " with cost " << edge.path.size() << std::endl;
	}
	for (const auto& e : graph.abstractEdges) {
		std::cerr << "  ABEDGE: " << e.from << " to " << e.to << " p: " << e.path.size() << std::endl;
		const auto& f = graph.abstractNodes[e.from];
		const auto& t = graph.abstractNodes[e.to];
		std::cerr << "    CENTER: " << f.center.first << "," << f.center.second << " to: " << t.center.first << "," << t.center.second << std::endl;
		std::cerr << "    BASE F:" << f.baseCenterNode << " FROM: " << graph.baseNodes[f.baseCenterNode].first << "," << graph.baseNodes[f.baseCenterNode].second << std::endl;
		std::cerr << "    BASE T:" << t.baseCenterNode << " TO  : " << graph.baseNodes[t.baseCenterNode].first << "," << graph.baseNodes[t.baseCenterNode].second << std::endl;
		for (const auto& p : e.path) {
			if (tempGrid[p.second][p.first] == (GridToGraph::XPND<<2)) continue;
			tempGrid[p.second][p.first] = GridToGraph::XPND<<1;
			std::cerr << "      " << p.first << "," << p.second << std::endl;
		}
		if (f.baseCenterNode != -1) {
			tempGrid[ graph.baseNodes[f.baseCenterNode].second] [graph.baseNodes[f.baseCenterNode].first] = GridToGraph::XPND<<2;
			std::cerr <<"##SET F " << graph.baseNodes[f.baseCenterNode].first <<","
					<< graph.baseNodes[f.baseCenterNode].second << std::endl;
		}
		if (t.baseCenterNode != -1) {
			tempGrid[graph.baseNodes[t.baseCenterNode].second] [graph.baseNodes[t.baseCenterNode].first] = GridToGraph::XPND<<2;
			std::cerr <<"##SET F" << graph.baseNodes[t.baseCenterNode].first << ","
					<< graph.baseNodes[t.baseCenterNode].second << std::endl;
		}
	}
	std::cerr << "\nBase Nodes:  " << graph.baseNodes.size() << std::endl;
	int i=0;
	for (const auto& node : graph.baseNodes) {
		std::cerr << "  " << i++ << " " << node.first <<"," << node.second << std::endl;
	}

	std::cerr << "NODES: " << graph.baseNodes.size() << std::endl
			<< "DENDS: " << graph.deadEnds.size() << std::endl
			<< "EDGES: " << graph.baseEdges.size() << std::endl
			<< std::endl
			<< "ABNOD: " << graph.abstractNodes.size() << std::endl
			<< "ABEDG: " << graph.abstractEdges.size() << std::endl;

	makeTGA("GRID_FULL.tga", tempGrid);
	for (int i=0; i < graph.baseEdges.size(); ++i) {
		const Edge& e = graph.baseEdges[i];
		std::cerr << "Path: " << i << " F:" << e.from << " T:" << e.to << (e.toDeadEnd ? " DEAD" : "")<< std::endl;
		if (e.toDeadEnd) continue;
		for (const auto& p : e.path) {
			std::cerr << "  " << p.first << "," << p.second << std::endl;
		}
	}

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

	bool toggle = true;
	std::cerr << "NAV GRID (closet absNode/baseNode)" << std::endl;
	std::cerr << "    ";
	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		if (col > 9) std::cerr << col << "   ";
		else std::cerr << col << "    ";
	}
	std::cerr << std::endl;
	for (int row=0; row < graph.infoGrid.size();) {

		if (toggle) {
			if (row > 9) std::cerr << row << "  ";
			else std::cerr << row << "   ";
			for (int col=0; col < graph.infoGrid[0].size(); ++col) {
				if (graph.infoGrid[row][col]&WALL) {
					std::cerr << "     ";
					continue;
				}
                Point p = {col, row};
                auto it = std::find(graph.baseNodes.begin(), graph.baseNodes.end(), p);
                if (it == graph.baseNodes.end()) {
					std::cerr << "     ";
                	continue;
                }

                int baseIdx = std::distance(graph.baseNodes.begin(), it);
                if (baseIdx >= 10) {
					std::cerr << baseIdx ;
				}
				else {
					std::cerr << baseIdx << " ";
				}
                int abIdx=0;
                for (AbstractNode abNod : graph.abstractNodes) {
                	if (abNod.baseCenterNode == baseIdx) {
                		if (abIdx >= 10) {
                			std::cerr << "/" << abIdx;
                		}
                		else {
                			std::cerr << "/" << abIdx << " ";
                		}
                		abIdx = -1;
                		break;
                	}
                	++abIdx;
                }
                if (abIdx != -1) {
                	std::cerr << "   ";
                }
            }
			std::cerr << std::endl;
			toggle = false;
			continue;
		}
		std::cerr << "    ";

		toggle = true;

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
		++row;
	}

	std::cerr << "ABSTRACT NODES (abnod / base center)" << std::endl;
	for (int col=0; col < graph.infoGrid[0].size(); ++col) {
		if (col > 9) std::cerr << col << "   ";
		else std::cerr << col << "    ";
	}
	for (int row=0; row < graph.infoGrid.size(); ++row) {

		if (row > 9) std::cerr << row << "   " << std::endl;
		else std::cerr << row << "    " << std::endl;

		for (int col=0; col < graph.infoGrid[0].size(); ++col) {
			bool found = false;
			int nidx = 0;
			for (const auto& n : graph.abstractNodes) {
				const auto& p = graph.baseNodes[n.baseCenterNode];
				if (p.first == col && p.second == row) {
					int b = n.baseCenterNode;
					if (b >= 10) {
						std::cerr << nidx << "/" << b << " ";
					}
					else {
						std::cerr << nidx << "/" << b << "  ";
					}
					found = true;
					break;
				}
				++nidx;
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
		std::cerr << "FOR ABEDGE: " << abEdge
				<< " F:" << graph.abstractEdges[abEdge].from << "(bNode " << graph.abstractNodes[graph.abstractEdges[abEdge].from].baseCenterNode << ")"
				<< " T:" << graph.abstractEdges[abEdge].to << "(bNode " << graph.abstractNodes[graph.abstractEdges[abEdge].to].baseCenterNode << ")"
				<< " T:" << graph.abstractEdges[abEdge].to
				<< std::endl;
		std::cerr << "   ";
		++abEdge;
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
