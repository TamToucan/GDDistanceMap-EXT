#ifndef DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_
#define DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_

#include <vector>
#include <utility>

namespace GridToGraph {

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;
using Edge = std::pair<Point, Point>;

// Take in 0 = wall, 1 = floor
void makeGraph(Grid& floorGrid);

std::vector<Point> detectDeadEnds(const Grid& grid);

std::vector<Point> detectNodes(const Grid& grid);

std::vector<Edge> findEdges(const Grid& grid, const std::vector<Point>& nodes, const std::vector<Point>& deadEnds);

}

#endif /* DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_ */
