#include <iostream>
#include <string>
#include "GridToGraph.hpp"
#include "FlowField.hpp"

struct FlowField::SubGrid subgrid;

void debugFlow(FlowField::SubGrid subGrid)
{
	int cols = subGrid.width;
	int rows = subGrid.height;
	const auto& flowField = subGrid.getFlow(0);
	for (int y = 0; y < rows; ++y) {
		for (int x = 0; x < cols; ++x) {
			int idx = FlowField::SubGrid::indexFor(x, y, cols);
			uint16_t costDir = flowField[idx];
			int cost = costDir >> 8;
			int dir = costDir & 0xFF;
			std::cout << (dir < 10 ? "  " : (dir<100 ? " " : "")) << dir << " ";
        }
		std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
#if 1
	auto grid = GridToGraph::readGridFromFile("D:/tmp/GRID.txt");
	auto graph = GridToGraph::makeGraph(grid);
	return 0;
#else
	auto grid = GridToGraph::readGridFromFile("D:/tmp/GRID.txt");
	subgrid.width = 13;
	subgrid.height = 11;
	subgrid.offsetX = 2;
	subgrid.offsetY = 1;
	auto W = GridType::WALL;
	subgrid.grid = {
W, W, W, W, W, W, W, W, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, W, W, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, W, W, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
W, 0, 0, W, W, W, W, 0, 0, 0, 0, 0, 0,
W, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, W, 0,
W, W, 0, 0, 0, 0, 0, 0, 0, 0, 0, W, W,
W, W, 0, 0, 0, 0, 0, 0, 0, 0, W, W, W,
W, W, W, 0, 0, 0, 0, 0, 0, W, W, W, W,
W, W, W, W, W, W, W, W, 0, W, W, W, W,
	};
	std::vector< std::pair<int,int>> sinks = { { 7,9 }, { 8,10 }, { 6,9 }, { 5,9 }, { 4,9 }, { 8,9 }, { 3,9 } };

	const auto costDirFlow = FlowField::generateFlowFieldDial(subgrid, sinks);
	subgrid.costFlowFields.push_back({ 0, costDirFlow });
	debugFlow(subgrid);
#endif
	return 1;
}