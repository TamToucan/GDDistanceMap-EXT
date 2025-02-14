#ifndef DISTANCEMAP_SRC_GRIDTYPES_HPP_
#define DISTANCEMAP_SRC_GRIDTYPES_HPP_

#include <vector>
#include <utility>

namespace GridType {

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;
using Path = std::vector<Point>;

    // Directions for 8 neighbouring cells
const static std::vector<GridType::Point> directions = {
		{-1, 0}, {1, 0}, {0, -1}, {0, 1}, // Up, Down, Left, Right
		{-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Diagonals
};




}

#endif /* DISTANCEMAP_SRC_GRIDTYPES_HPP_ */
