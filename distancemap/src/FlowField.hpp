#ifndef DISTANCEMAP_SRC_FLOWFIELD_HPP_
#define DISTANCEMAP_SRC_FLOWFIELD_HPP_

#include <unordered_map>
#include <optional>
#include <cstdint>

#include "GridTypes.hpp"

// Forward Dec
namespace GridToGraph {
	struct Graph;
}


namespace FlowField {

	struct SubGrid {
		std::vector<int> grid;  // Flat grid array to be used as the input for generateFlowFieldDial.
		std::vector<std::pair<int, std::vector<uint16_t>> > costFlowFields;
		inline const std::vector<uint16_t>& getFlow(int adjZone) const {
			static const std::vector<uint16_t> emptyFlow = {};
			for (const auto& pair : costFlowFields) if (pair.first == adjZone) return pair.second;
			return emptyFlow;
		}

		int width;
		int height;
		int offsetX;  // Global x coordinate of the subgrid’s top-left cell.
		int offsetY;  // Global y coordinate of the subgrid’s top-left cell.
		bool isInside(int x, int y) const { return (x >= 0 && x < width && y >= 0 && y < height); }
		// Using a flat index for arrays.
		static inline int indexFor(int x, int y, int cols) { return y * cols + x; }
	};

void generateFlowGrids(GridToGraph::Graph& graph);

}

#endif /* DISTANCEMAP_SRC_FLOWFIELD_HPP_ */
