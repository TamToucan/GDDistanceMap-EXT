#ifndef DISTANCEMAP_SRC_FLOWFIELD_HPP_
#define DISTANCEMAP_SRC_FLOWFIELD_HPP_

#include <iostream>
#include <unordered_map>
#include <optional>
#include <cstdint>

#include "GridTypes.hpp"

// Forward Dec
namespace GridToGraph {
	struct Graph;
}


namespace FlowField {

	constexpr uint8_t NO_DIR = 127;
	constexpr uint8_t SINK_BIT = 0x80;

	struct SubGrid {
		std::vector<int> grid;  // Flat grid array to be used as the input for generateFlowFieldDial.
		// List for { adjacent zone, flow field} pairs
		std::vector<std::pair<int, std::vector<uint16_t>> > costFlowFields;

		int width;
		int height;
		int offsetX;  // Global x coordinate of the subgrids top-left cell.
		int offsetY;  // Global y coordinate of the subgrids top-left cell.
		bool isInside(int x, int y) const { return (x >= 0 && x < width && y >= 0 && y < height); }

		// Find the flowField for the given adjacent zone.
		inline const std::vector<uint16_t>& getFlow(int adjZone) const {
			static const std::vector<uint16_t> emptyFlow = {};
			for (const auto& pair : costFlowFields) if (pair.first == adjZone) return pair.second;
			std::cerr << "ERROR: " << adjZone << " not an adjacent zone" << std::endl;
			return emptyFlow;
		}

		// gethe <8bit cost> | <sinkBit> <direction Index>
		inline uint16_t getCostFlow(int x, int y, const std::vector<uint16_t>& flow) const {
			return flow[indexFor(x - offsetX, y - offsetY, width)];
		}
		// Using a flat index for arrays.
		static inline int indexFor(int x, int y, int cols) { return y * cols + x; }
	};

void generateFlowGrids(GridToGraph::Graph& graph);

}

#endif /* DISTANCEMAP_SRC_FLOWFIELD_HPP_ */
