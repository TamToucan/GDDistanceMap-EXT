#include <vector>
#include <unordered_map>
#include <optional>
#include <queue>
#include <future>
#include <cmath>
#include <iostream>
#include <cstdint>
#include <vector>
#include <limits>
#include <array>
#include <cassert>

#include "FlowField.hpp"
#include "GridTypes.hpp"
#include "GridToGraph.hpp"

namespace FlowField {
	// Maximum cost (our cost type is uint8_t, so 255 is max).
	constexpr uint8_t MAX_COST = 255;
	constexpr uint8_t NO_DIR = 127;
	constexpr uint8_t SINK_BIT = 0x80;

	// Movement in 8 directions8: N, NE, E, SE, S, SW, W, NW.
	constexpr std::array<int, 8> dx = { 0,  1, 1,  1,  0, -1, -1, -1 };
	constexpr std::array<int, 8> dy = { -1, -1,0,  1,  1,  1,  0, -1 };

	// Given uniform cost, each step costs 1.
	constexpr uint8_t STEP_COST = 1;


// Extract a subgrid for the target zone.
// zoneGrid: global vector of vector of GridPointInfo (size = rows x cols)
// origGrid: original grid with tile flags (e.g. WALL), same dimensions as zoneGrid.
// targetZone: the abstract node index that identifies the zone.
SubGrid extractZoneSubGrid(const std::vector<std::vector<GridType::GridPointInfo>>& zoneGrid,
		const std::vector<std::vector<int>>& origGrid,
		int targetZone)
{
	int rows = zoneGrid.size();
	int cols = zoneGrid[0].size();

	// First, find the bounding box of cells that belong to the target zone.
	int minX = cols;
	int minY = rows;
	int maxX = -1;
	int maxY = -1;
	for (int y = 1; y < rows-1; ++y) {
		for (int x = 1; x < cols-1; ++x) {
			if (zoneGrid[y][x].closestAbstractNodeIdx == targetZone) {
				if (x < minX) minX = x;
				if (x > maxX) maxX = x;
				if (y < minY) minY = y;
				if (y > maxY) maxY = y;
			}
		}
	}
	std::cerr << "Zone: " << targetZone << " bounding box: (" << minX << ", " << minY << ") to (" << maxX << ", " << maxY << ")" << std::endl;

	// Validate that we found at least one cell for the zone.
	if (maxX < minX || maxY < minY)
		throw std::runtime_error("The target zone has no cells in the grid.");

	int subWidth = maxX - minX + 1;
	int subHeight = maxY - minY + 1;

	SubGrid subGrid;
	subGrid.width = subWidth;
	subGrid.height = subHeight;
	subGrid.offsetX = minX;
	subGrid.offsetY = minY;
	subGrid.grid.resize(subWidth * subHeight, GridType::WALL); // Default: mark as WALL.

	// Fill subGrid: For each global cell in the bounding box,
	// if it belongs to targetZone and is not a wall in origGrid, mark it as walkable (0).
	// Otherwise, mark it as WALL.
	for (int y = minY; y <= maxY; ++y) {
		for (int x = minX; x <= maxX; ++x) {
			int localX = x - minX;
			int localY = y - minY;
			int idx = localY * subWidth + localX;
			// Check: belongs to target zone and not a wall.
			if (zoneGrid[y][x].closestAbstractNodeIdx == targetZone &&
				(origGrid[y][x] & GridType::WALL) == 0)
			{
				subGrid.grid[idx] = 0;  // Walkable.
			}
			else {
				subGrid.grid[idx] = GridType::WALL;  // Not part of the zone or is a wall.
			}
		}
	}

	return subGrid;
}

std::vector<std::pair<int, int>> convertSinksToLocal(const GridType::BoundaryCells& globalSinks, const SubGrid& subGrid)
{
	std::vector<std::pair<int, int>> localSinks;
	for (const auto& sink : globalSinks) {
		int gx = sink.first;
		int gy = sink.second;
		// Only include sinks that are within the bounding box.
		if (gx >= subGrid.offsetX && gx < subGrid.offsetX + subGrid.width &&
			gy >= subGrid.offsetY && gy < subGrid.offsetY + subGrid.height)
		{
			int localX = gx - subGrid.offsetX;
			int localY = gy - subGrid.offsetY;
			localSinks.emplace_back(localX, localY);
			std::cerr << "SINK: " << gx << "," << gy << " => " << localX <<"," << localY<< std::endl;
		}
	}
	return localSinks;
}


	// ---------------------------------------------------------------------------
	// generateFlowFieldDial()
	// grid: a flat vector for grid, row-major, where each cell is an int (WALL bit mask)
	// sinks: list of sink cells (boundary cells) as pairs of (x, y)
	// costField and dirField: flat arrays of size rows*cols that will be written to.
	// rows, cols: dimensions of the grid.
	// isWalkable: lambda function taking (x,y) and grid, returns true for non-wall cells.
template <typename WalkableFn>
std::vector<uint16_t> generateFlowFieldDial(const std::vector<int>& grid,
		const std::vector<std::pair<int, int>>& sinks,
		int rows, int cols,
		WalkableFn isWalkable)
{
	// Total number of cells.
	const int cellCount = rows * cols;
	std::vector<uint16_t> costFlowField;
	costFlowField.assign(cellCount, MAX_COST<<8 | NO_DIR);

	// Buckets: one bucket per possible cost value from 0 up to MAX_COST.
	// We store indices (flat indices for cells) in each bucket.
	std::vector<int> buckets;
	std::vector<int> bucket_offsets(MAX_COST + 2);

	// Count sink positions per bucket (bucket 0 only)
	bucket_offsets[1] = sinks.size(); // All sinks go to bucket 0

	// Build flat buckets
	buckets.reserve(sinks.size());
	for (const auto& [x, y] : sinks) {
		int idx = SubGrid::indexFor(x, y, cols);
		buckets.push_back(idx);
		costFlowField[idx] = 0 | NO_DIR | SINK_BIT;
	}
	int sinksToGo = sinks.size();

	std::cerr << "DIAL: " << cols << "x" << rows << " sinks:" << sinks.size() << std::endl;
	// Dial's algorithm: process each bucket, in increasing cost order.
	for (uint8_t cost = 0; cost <= MAX_COST && sinksToGo; ++cost) {
		const int start = bucket_offsets[cost];
		const int end = bucket_offsets[cost + 1];

		for (int i = start; i < end; ++i) {
			int curIdx = buckets[i];
			std::cerr << "  Bucket[cost:" << static_cast<int>(cost) << "] => "<< curIdx << std::endl;

			int x = curIdx % cols;
			int y = curIdx / cols;
			if (costFlowField[curIdx]&SINK_BIT) {
				--sinksToGo;
				std::cerr << "  DONE SINK: " << x << "," << y << " toGo: " << sinksToGo << std::endl;
			}

			std::cerr << "  cost:" << static_cast<int>(cost) << " xy:" << x << "," << y << " idx:" << curIdx << std::endl;
			// Explore the 8 neighbors
			for (int d = 0; d < 8; ++d) {
				int nx = x + dx[d];
				int ny = y + dy[d];

				// Check walkability: any non-wall cell is walkable.
				if (!isWalkable(nx, ny, grid)) continue;
				int nIdx = SubGrid::indexFor(nx, ny, cols);

				std::cerr << "    walkable:" << nx << "," << ny << " cost:" << static_cast<int>(cost) << " curCost:" << (costFlowField[nIdx] >> 8) << std::endl;
				// New cost is current cost + STEP_COST.
				uint8_t newCost = cost + STEP_COST;
				// If new cost is better than the neighbor's cost, update it.
				if (newCost < costFlowField[nIdx]>>8) {
					// Store the direction that would bring this neighbor closer to a sink.
					// We store the reverse direction (the direction from neighbor back to the current cell)
					// so that following the vector will lead from any cell toward one of the sinks.
					// (d + 4) % 8 gives the opposite direction.
					costFlowField[nIdx] = newCost << 8 | ((d + 4) % 8);

					std::cerr << "      store costDir: " << std::hex << costFlowField[nIdx] << std::dec << std::endl;

					// Only insert if we haven't exceeded MAX_COST.
					if (newCost <= MAX_COST) {
						buckets.push_back(nIdx);
						bucket_offsets[newCost + 1]++; // Expand next bucket
					}
				}
			}
		}
	}
	return costFlowField;
}

#if 0
std::vector<uint16_t> precomputeBoundaryDF(const GridType::BoundaryCells& cells, const SubGrid& subGrid)
{
	std::vector<uint16_t> costFlowField;

	std::queue<std::tuple<int, int, uint8_t>> q;
	costFlowField.assign(subGrid.width*subGrid.height, MAX_COST<<8 | NO_DIR);
	for (auto [x, y] : cells) {
		int idx = SubGrid::indexFor(x, y, subGrid.width);
		costFlowField[idx] = (NO_DIR << 8) | 0; // 0 distance at boundary
		q.push({ x, y, 0 });
	}

	while (!q.empty()) {
		auto [x, y, dist] = q.front(); q.pop();
		for (int d = 0; d < 8; ++d) {
			int nx = x + dx[d];
			int ny = y + dy[d];
			if (!subGrid.isInside(nx, ny)) continue;

			int nIdx = SubGrid::indexFor(nx, ny, subGrid.width);
			int newDist = dist + 1;
			int oldDist = costFlowField[nIdx] >> 8;
			if (newDist < oldDist) {
				costFlowField[nIdx] = newDist<<8 | d;
				q.push({ nx, ny, newDist });
			}
		}
	}
	return costFlowField;
}

#endif


//#########################################################################################

void generateFlowGrids(GridToGraph::Graph& graph)
{
	std::cout << "#==============# GENERATE FLOW GRIDS" << std::endl;

	const int rows = graph.infoGrid.size();
	const int cols = graph.infoGrid[0].size();

	std::cerr << "## SUBS: AbstractLevels: " << graph.abstractLevels.size() << std::endl;
	for (int i = 0; i < graph.abstractLevels.size(); ++i) {
		auto& ablv = graph.abstractLevels[i];
		for (int zi = 0; zi < ablv.zones.size(); ++zi) {
			std::cerr << "     zone: " << zi << std::endl;
			auto subgrid = extractZoneSubGrid(ablv.zoneGrid, graph.infoGrid, zi);
			ablv.subGrids.push_back(subgrid);
		}
		std::cerr << "  AbstractLevel: " << i << " subgrids: " << ablv.subGrids.size() << std::endl;
	}

	std::cerr << "## FLOW: AbstractLevels: " << graph.abstractLevels.size() << std::endl;
	int levIdx = 0;
#if 0
	const int fullSubGridStart = 0; // (graph.abstractLevels.size() + 1) / 2;
	while (levIdx != fullSubGridStart)
	{
		auto& ablv = graph.abstractLevels[levIdx];
		for (int zi = 0; zi < ablv.zones.size(); ++zi) {
			auto& subGrid = ablv.subGrids[zi];
			const auto& adjZones = ablv.zones[zi].adjacentZones;
			std::cerr << "    Zone: " << zi << " adjZones: " << adjZones.size() << std::endl;
			GridType::BoundaryCells allSinks;
			for (int ni = 0; ni < adjZones.size(); ++ni) {
				int adjZone = adjZones[ni];
				const auto& bcells = ablv.zones[zi].zoneBoundaryCellMap[adjZone];
				auto localSinks = convertSinksToLocal(bcells, subGrid);
				allSinks.insert(localSinks.begin(), localSinks.end());
			}
			std::cerr << "  SMALL AbstractLevel: " << levIdx << " Z: " << zi << std::endl;
			subGrid.costFlowField = precomputeBoundaryDF(allSinks, subGrid);
		}
		++levIdx;
	}
#endif


	for (; levIdx < graph.abstractLevels.size(); ++levIdx) {
		auto& ablv = graph.abstractLevels[levIdx];
	    std::cerr << "  AbstractLevel: " << levIdx << " zones: " << ablv.zones.size() << std::endl;
		for (int zi = 0; zi < ablv.zones.size(); ++zi) {
			auto& subGrid = ablv.subGrids[zi];
			const auto& adjZones = ablv.zones[zi].adjacentZones;
			std::cerr << "    Zone: " << zi << " adjZones: " << adjZones.size() << std::endl;
			for (int ni = 0; ni < adjZones.size(); ++ni) {
				int adjZone = adjZones[ni];
				const auto& boundaryCells = ablv.zones[zi].zoneBoundaryCellMap[adjZone];
				auto localSinks = convertSinksToLocal(boundaryCells, subGrid);

				std::cerr << "        subGrid: " << subGrid.offsetX<<","<<subGrid.offsetY <<" "<<subGrid.width<<"x"<<subGrid.height
					<< " sinks: " << localSinks.size() << std::endl;
				std::cerr << "== lv:" << levIdx << " zn:" << zi << " adj:" << ni <<" sinks:" << localSinks.size() << std::endl;
				subGrid.costFlowField = generateFlowFieldDial(subGrid.grid, localSinks, subGrid.height, subGrid.width,
					// Lambda: non-wall (walkable) check.
					[=](int x, int y, const std::vector<int>& grid) -> bool {
						if (!subGrid.isInside(x, y)) return false;
						int idx = SubGrid::indexFor(x,y, subGrid.width);
						return (grid[idx] & GridType::WALL) == 0;
					});
			}
		}
	}
}

}




