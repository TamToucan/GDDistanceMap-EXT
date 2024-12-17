#include "RogueCave.hpp"

#include <cstdlib>
#include <iostream>

namespace PCG {

RogueCave::RogueCave(int width, int height) :
			mWidth(width),
			mHeight(height)
	{
    grid.resize(mHeight, std::vector<int>(mWidth, TILE_WALL));
    grid2.resize(mHeight, std::vector<int>(mWidth, TILE_WALL));
	std::cerr << "## CREATE ROGUE CAVE " << mWidth << "x" << mHeight << std::endl;
}

void RogueCave::addGeneration(Util::IntRange b3, Util::IntRange b5, Util::IntRange s3, Util::IntRange s5, int reps) {
	mGenerations.push_back( Generation(b3, b5, s3, s5, reps) );
}

std::pair<int, int> RogueCave::count_neighbors(int x, int y) {
    int count_5x5 = 0;
    int count_3x3 = 0;

    // Iterate through the 5x5 grid
    for (int dy = -2; dy <= 2; ++dy) {
    	for (int dx = -2; dx <= 2; ++dx) {
    		// Don't check the 0,0 point
    		if ((dx|dy) == 0) continue;
    		int nx = x + dx;
    		int ny = y + dy;
    		// Bounds check for the grid
    		if (nx >= 0 && nx < mWidth && ny >= 0 && ny < mHeight) {
    			if (grid[ny][nx] != TILE_FLOOR) {
    				++count_5x5; // Increment 5x5 counter

    				// Additional bounds check for 3x3 grid
    				if (std::abs(dx) <= 1 && std::abs(dy) <= 1) {
    					++count_3x3; // Increment 3x3 counter
    				}
    			}
    		}
    		// Count border as walls
    		else {
    			count_5x5++;
    			if (std::abs(dx) <= 1 && std::abs(dy) <= 1) {
    				++count_3x3;
    			}
    		}
    	}
    }

    return {count_3x3, count_5x5}; // Return a pair of counts
}

// Perform one generation of cellular automaton
void RogueCave::generation(const RogueCave::Generation& params) {
	std::cerr << "########################### GENERATE ROGUE CAVE ##########################" << std::endl;
    for (int y = 0; y < mHeight; ++y) {
        for (int x = 0; x < mWidth; ++x) {
            grid2[y][x] = grid[y][x];
        }
    }

    for (int y = 0; y < mHeight ; ++y) {
        for (int x = 0; x < mWidth ; ++x) {
            std::pair<int,int> neighbors = count_neighbors(x,y);
            int n3x3 = neighbors.first;
            int n5x5 = neighbors.second;

            if (grid[y][x] == TILE_WALL) {
            	if ((n3x3 < params.survive3.min || n3x3 > params.survive3.max)
            	&& (n5x5 < params.survive5.min || n5x5 > params.survive5.max)) {
            		grid2[y][x] = TILE_FLOOR;
            	}
            }
            else {
            	if ((n3x3 >= params.born3.min && n3x3 <= params.born3.max)
            	 || (n5x5 >= params.born5.min && n5x5 <= params.born5.max)) {
            		grid2[y][x] = TILE_WALL;
            	}
            }
        }
    }

    // Update the grid
    for (int yi = 0; yi < mHeight; ++yi) {
        for (int xi = 0; xi < mWidth; ++xi) {
            grid[yi][xi] = grid2[yi][xi];
        }
    }
}

std::vector<std::vector<int>>& RogueCave::generate() {
	std::cerr << "## GENERATE ROGUE " << "  " << grid.size() << "x" << grid[0].size() << std::endl;
    for (const auto& params : mGenerations) {
        for (int i = 0; i < params.reps; ++i) {
        	std::cerr << "===== Rep: " << i << " of " << params.reps
        			<< " B3x3:" << params.born3 << " B5x5:" << params.born5
        			<< " S3x3:" << params.survive3 << " S5x5:" << params.survive5
					<< std::endl;
            generation(params);
        }
    }
	std::cerr << "## RETURN " << "  " << grid.size() << "x" << grid[0].size() << std::endl;

    return grid;
}

}
