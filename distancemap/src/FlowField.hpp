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

// Flow field data packed as 32 bits: top 16 bits = cost, bottom 16 bits = direction index
using FlowFieldData = uint32_t;

struct SparseFlowField {
    std::unordered_map<GridType::Point, FlowFieldData, GridType::PairHash> flowData;
    // Returns
    std::optional<std::pair<int, int>> unpack(const GridType::Point& point) const;
};

void generateFlowFieldsParallel(const GridToGraph::Graph* graph, std::vector<SparseFlowField>& flowFields);

}

#endif /* DISTANCEMAP_SRC_FLOWFIELD_HPP_ */
