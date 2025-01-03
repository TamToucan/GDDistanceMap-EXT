/*
 * WallDistanceGrid.cpp
 *
 *  Created on: 31 Dec 2024
 *      Author: tam
 */

#include "WallDistanceGrid.hpp"

#include <vector>
#include <queue>
#include <limits>
#include <iostream>

namespace DistanceMap {

std::vector<std::vector<int>> makeWallDistanceGrid(const std::vector<std::vector<int>>& grid)
{
    int rows = grid.size();
    int cols = grid[0].size();

    // Directions for moving (up, down, left, right)
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    // Initialize the distance grid with a large value
    std::vector<std::vector<int>> distanceGrid(rows, std::vector<int>(cols, std::numeric_limits<int>::max()));

    // BFS queue to store cells (row, col)
    std::queue<std::pair<int, int>> bfsQueue;

    // Add all wall cells to the queue and initialize their distances
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (grid[r][c]) {
                distanceGrid[r][c] = 0;
                bfsQueue.push({r, c});
            }
        }
    }

    // Perform BFS
    while (!bfsQueue.empty()) {
        auto [current_r, current_c] = bfsQueue.front();
        bfsQueue.pop();

        // Current cell's distance
        int current_distance = distanceGrid[current_r][current_c];

        // Process neighbors
        for (const auto& [dr, dc] : directions) {
            int neighbor_r = current_r + dr;
            int neighbor_c = current_c + dc;

            // Ensure neighbor is within bounds and update its distance if shorter
            if (neighbor_r >= 0 && neighbor_r < rows && neighbor_c >= 0 && neighbor_c < cols) {
                if (distanceGrid[neighbor_r][neighbor_c] > current_distance + 1) {
                    distanceGrid[neighbor_r][neighbor_c] = current_distance + 1;
                    bfsQueue.push({neighbor_r, neighbor_c});
                }
            }
        }
    }

    return distanceGrid;
}

} // namespace
