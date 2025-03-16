#include <iostream>
#include <vector>
#include <iomanip>

// Alias for convenience

namespace Algo {

using BinaryImage = std::vector<std::vector<int>>;

namespace {

// Count the number of non-zero neighbors around a pixel
int countNeighbors(const BinaryImage& image, int x, int y) {
    int count = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            if (image[x + dx][y + dy] > 0) ++count;
        }
    }
    return count;
}

// Count transitions from 0 to 1 in the 8-neighborhood (clockwise)
int countTransitions(const BinaryImage& image, int x, int y) {
    int transitions = 0;
    int neighbors[8] = {
        image[x - 1][y],     // P2
        image[x - 1][y + 1], // P3
        image[x][y + 1],     // P4
        image[x + 1][y + 1], // P5
        image[x + 1][y],     // P6
        image[x + 1][y - 1], // P7
        image[x][y - 1],     // P8
        image[x - 1][y - 1]  // P9
    };

    for (int i = 0; i < 8; ++i) {
        if (neighbors[i] == 0 && neighbors[(i + 1) % 8] == 1) {
            ++transitions;
        }
    }
    return transitions;
}

// Perform one thinning iteration
bool thinningIteration(BinaryImage& image, bool firstSubIteration)
{
    std::vector<std::pair<int, int>> toBeRemoved;

    for (int x = 1; x < image.size() - 1; ++x) {
        for (int y = 1; y < image[0].size() - 1; ++y) {
            // Process only foreground pixels
            if (image[x][y] != 1) continue;

            int neighbors = countNeighbors(image, x, y);
            int transitions = countTransitions(image, x, y);
            int P2 = image[x - 1][y];     // North
            int P4 = image[x][y + 1];     // East
            int P6 = image[x + 1][y];     // South
            int P8 = image[x][y - 1];     // West

            // Check the Zhang-Suen conditions
            if (neighbors >= 2 && neighbors <= 6 &&
                transitions == 1 &&
                (firstSubIteration ? (P2 * P4 * P6 == 0 && P4 * P6 * P8 == 0)
                                   : (P2 * P4 * P8 == 0 && P2 * P6 * P8 == 0))) {
                toBeRemoved.emplace_back(x, y);
            }
        }
    }

    // Remove marked pixels
    for (const auto& pixel : toBeRemoved) {
        image[pixel.first][pixel.second] = 0;
    }

    return !toBeRemoved.empty();
}

} // anonymouse


//
// Zhang-Suen Thinning Algorithm
//
void ZSThinning(BinaryImage& grid)
{
    bool changed;
    do {
        changed = thinningIteration(grid, true);  // First sub-iteration
        changed |= thinningIteration(grid, false); // Second sub-iteration
    } while (changed);
}

}
