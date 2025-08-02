#include <iostream>
#include <string>
#include "GridToGraph.hpp"
#include "Router.hpp"
#include "FlowField.hpp"

#include "GDTracker.hpp"
#include "GDDistanceMap.hpp"

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

std::pair<float, float> computeDirection(float angleDeg) {
	const double MYPI = 3.14159265358979323846;
	double radians = angleDeg * (MYPI / 180.0);
	return { std::cos(radians), std::sin(radians) };
}

int main(int argc, char** argv)
{
#if 1
	auto grid = GridToGraph::readGridFromFile("GRID.txt");
	auto graph = GridToGraph::makeGraph(grid);

	Router::Info info;
	info.mCaveHeight = 32;
	info.mCellWidth = 8;
	info.mCellHeight = 8;
	Vector2 from(648, 827);
	Vector2 to(1956, 54);
	Router::RouteCtx* ctx = new Router::RouteCtx();
	do {
		ctx->from.first = from.x;
		ctx->from.second = from.y;
		ctx->to.first = to.x;
		ctx->to.second = to.y;
		ctx->type = 0;
		float ang = Router::getAngle(graph, info, ctx, from, to, 0);
		GridType::Point mv = computeDirection(ang);
		ctx->next.first += mv.first * 8;;
		ctx->next.second += mv.second * 8;
	} while (from != to);
	delete ctx;

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
	std::cerr << "FlowField generated" << std::endl;
	return 1;
}