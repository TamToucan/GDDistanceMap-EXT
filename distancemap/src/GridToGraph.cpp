#include <iostream>
#include <vector>

using Pattern = std::vector<std::vector<int>>;
namespace {
#include <vector>

using Pattern = std::vector<std::vector<int>>;

// Define all patterns for one rotation (0 degrees) using 0, 1, and 2
const std::vector<Pattern>& getBasePatterns() {
    static std::vector<Pattern> patterns;

    if (patterns.size() != 0) {
    	return patterns;

    }
    // 4-Way Junction (X-Junction)
    patterns.push_back({
        {2, 1, 2, 2},
        {1, 1, 1, 2},
        {2, 1, 2, 2},
        {2, 2, 2, 2}
    });

    // T-Junctions
    // T-Junction (Missing Bottom Path)
    patterns.push_back({
        {2, 1, 2, 2},
        {1, 1, 1, 2},
        {2, 2, 2, 2},
        {2, 2, 2, 2}
    });

    // T-Junction (Missing Right Path)
    patterns.push_back({
        {2, 1, 2, 2},
        {1, 1, 2, 2},
        {2, 1, 2, 2},
        {2, 2, 2, 2}
    });

    // T-Junction (Missing Top Path)
    patterns.push_back({
        {2, 2, 2, 2},
        {2, 1, 2, 2},
        {1, 1, 1, 2},
        {2, 1, 2, 2}
    });

    // T-Junction (Missing Left Path)
    patterns.push_back({
        {2, 1, 2, 2},
        {2, 1, 1, 2},
        {2, 1, 2, 2},
        {2, 2, 2, 2}
    });

    // Y-Junctions
    // Y-Junction (Vertical Path with Left and Right)
    patterns.push_back({
        {2, 1, 2, 2},
        {1, 1, 1, 2},
        {2, 1, 2, 2},
        {2, 2, 2, 2}
    });

    // Y-Junction (Left, Bottom, and Top)
    patterns.push_back({
        {2, 1, 2, 2},
        {1, 1, 2, 2},
        {2, 1, 2, 2},
        {2, 1, 2, 2}
    });

    return patterns;
}

bool matchesPattern(const std::vector<std::vector<int>>& grid, int x, int y, const Pattern& pattern) {
	if (x > grid[0].size()-4 || y > grid.size()-4) return false;

    for (int i = 0; i < 4; ++i) {
        int gy = y + i;
        for (int j = 0; j < 4; ++j) {
            int gx = x + j;
            if ((pattern[i][j] != 2) && grid[gy][gx] != pattern[i][j]) {
                return false;
            }
        }
    }
    std::cerr << "MATCH " << x << "," << y << " with " << std::endl;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cerr << " " << (pattern[i][j]);
        }
        std::cerr << std::endl;
    }
    return true;
}


Pattern rotatePattern(const Pattern& pattern) {
    Pattern rotated(4, std::vector<int>(4, 0));
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            rotated[j][3 - i] = pattern[i][j]; // Rotate clockwise
        }
    }
    return rotated;
}

const std::vector<Pattern>& getAllPatterns() {
    static std::vector<Pattern> allPatterns;

    if (allPatterns.size() == 0) {
    	for (const auto& pattern : getBasePatterns()) {
        	Pattern current = pattern;
        	for (int r = 0; r < 4; ++r) {
            	allPatterns.push_back(current);
            	current = rotatePattern(current);
        	}
    	}
    }

    return allPatterns;
}

}

namespace GridToGraph {

std::vector<std::pair<int, int>> detectNodes(const std::vector<std::vector<int>>& grid)
		{
    std::vector<std::pair<int, int>> nodes;
    for (int y = 0; y <= grid.size() - 4; ++y) {
        for (int x = 0; x <= grid[0].size() - 4; ++x) {
            for (const auto& pattern : getAllPatterns()) {
                if (matchesPattern(grid, x, y, pattern)) {
                    nodes.emplace_back(x + 2, y + 2);
                    std::cerr << "==STORE " << x+2 << "," << y+2 << std::endl;
                    break;
                }
            }
        }
    }

    return nodes;
}

}
