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

	/*
CTX ===CREATE CONTEXT
CTX ===ADD CALLBACK
CTX: F: 4,3 C T: 28,15 C N: 5,3 C DIR: 0 FRM 300,250 TO  1830.51,986.754 FPNT: 4,3 TPNT: 28,15
CTX: F: 4,3 C T: 28,15 C N: 5,3 C DIR: 0 FRM 323.334,250 TO  1830.51,986.754 FPNT: 5,3 TPNT: 28,15
CTX: F: 5,3 C T: 28,15 C N: 6,3 C DIR: 0 FRM 346.667,250 TO  1830.51,986.754 FPNT: 5,3 TPNT: 28,15
CTX: F: 5,3 C T: 28,15 C N: 6,3 C DIR: 0 FRM 370,250 TO  1830.51,986.754 FPNT: 5,3 TPNT: 28,15
CTX: F: 5,3 C T: 28,15 C N: 6,3 C DIR: 0 FRM 393.334,250 TO  1830.51,986.754 FPNT: 6,3 TPNT: 28,15
CTX: F: 6,3 C T: 28,15 C N: 7,4 C DIR: 45 FRM 409.833,266.499 TO  1830.51,986.754 FPNT: 6,4 TPNT: 28,15
CTX: F: 6,3 C T: 28,15 C N: 7,4 C DIR: 45 FRM 426.332,282.998 TO  1830.51,986.754 FPNT: 6,4 TPNT: 28,15
CTX: F: 6,3 C T: 28,15 C N: 7,4 C DIR: 45 FRM 442.831,299.498 TO  1830.51,986.754 FPNT: 6,4 TPNT: 28,15
CTX: F: 6,3 C T: 28,15 C N: 7,4 C DIR: 45 FRM 459.33,315.997 TO  1830.51,986.754 FPNT: 7,4 TPNT: 28,15
CTX: F: 7,4 C T: 28,15 C N: 8,4 C DIR: 0 FRM 482.664,315.997 TO  1830.51,986.754 FPNT: 7,4 TPNT: 28,15
CTX: F: 7,4 C T: 28,15 C N: 8,4 C DIR: 0 FRM 505.997,315.997 TO  1830.51,986.754 FPNT: 7,4 TPNT: 28,15
CTX: F: 7,4 C T: 28,15 C N: 8,4 C DIR: 0 FRM 529.331,315.997 TO  1830.51,986.754 FPNT: 8,4 TPNT: 28,15
CTX: F: 8,4 C T: 28,15 C N: 9,4 C DIR: 0 FRM 552.664,315.997 TO  1830.51,986.754 FPNT: 8,4 TPNT: 28,15
CTX: F: 8,4 C T: 28,15 C N: 9,4 C DIR: 0 FRM 575.997,315.997 TO  1830.51,986.754 FPNT: 8,4 TPNT: 28,15
CTX: F: 8,4 C T: 28,15 C N: 9,4 C DIR: 0 FRM 599.33,315.997 TO  1830.51,986.754 FPNT: 9,4 TPNT: 28,15
CTX: F: 9,4 C T: 28,15 C N: 10,5 C DIR: 45 FRM 615.829,332.496 TO  1830.51,986.754 FPNT: 9,5 TPNT: 28,15
CTX: F: 9,4 C T: 28,15 C N: 10,5 C DIR: 45 FRM 632.328,348.995 TO  1830.51,986.754 FPNT: 9,5 TPNT: 28,15
CTX: F: 9,4 C T: 28,15 C N: 10,5 C DIR: 45 FRM 648.827,365.494 TO  1830.51,986.754 FPNT: 10,5 TPNT: 28,15
*/
	Router::Info info;
	info.mCaveWidth = 34;
	info.mCaveHeight = 34;
	info.mCellWidth = 8;
	info.mCellHeight = 8;
	Vector2 from(300, 250);
	Vector2 to(1830, 986);
	Router::RouteCtx* ctx = new Router::RouteCtx();
	ctx->type = -1;
	int count = 100;
	do {
		float ang = Router::getAngle(graph, info, ctx, from, to, 0);
		std::pair<float, float> mv = computeDirection(ang);
		from.x += mv.first * 23;
		from.y += mv.second * 23;
		if (from.x > info.mCaveWidth*info.mCellWidth*8) {
			std::cerr << "from.x > info.mCaveWidth*info.mCellWidth*8 out of bounds" << std::endl;
			return -1;
		}
		if (from.y > info.mCaveHeight*info.mCellHeight*8) {
			std::cerr << "from.y > info.mCaveHeight*info.mCellHeight*8 out of bounds" << std::endl;
			return -1;
		}
		std::cerr << "CTV MV " << mv.first << "," << mv.second
			<< "  ang " << ang<< std::endl;
	} while ((ctx->from != ctx->to) && (--count > 0));
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