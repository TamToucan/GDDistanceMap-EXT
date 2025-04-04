#include <vector>
#include <unordered_map>
#include <optional>
#include <queue>
#include <future>
#include <cmath>
#include <iostream>

#include "FlowField.hpp"
#include "GridToGraph.hpp"

namespace FlowField {

using GridType::Point;


std::optional<std::pair<int, int>> SparseFlowField::unpack(const Point& point) const {
	auto it = flowData.find(point);
	if (it == flowData.end()) {
		return std::nullopt; // Indicate that the point is not in the flow field
	}
	uint8_t packedValue = it->second;
	int direction = (packedValue >> 5) & 0b111;
	int cost = packedValue & 0b11111;

	return std::make_pair(direction, cost);
}

// Packing and unpacking functions
uint32_t packFlowFieldData(int directionIndex, int cost) {
	return (directionIndex<<5) | (cost & 0b11111);
}

// Helper to calculate scaled cost
int calculateScaledCost(int distance, int maxDistance) {
    int d = (distance <= maxDistance) ? std::round((distance / maxDistance) * 31) : 31;
	std::cerr << "   SCALE:" << distance << " max: " << maxDistance << " res:" << d << std::endl;
	return d;
}

// BFS for flow field generation
void bfsFlowField(const GridType::Grid& grid, SparseFlowField& flowField, const std::vector<Point>& sources, int maxCost, int maxDistance)
{
    using QueueElement = std::pair<int, Point>; // (cost, point)
    std::cerr << "--BTS sz:" << sources.size() << " maxCost: " << maxCost << " maxDist: " << maxDistance << std::endl;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> pq;
    std::unordered_map<Point, int, GridType::PairHash> costMap;

    // Initialize priority queue with source points
    for (const auto& source : sources) {
        pq.push({0, source});
        costMap[source] = 0;
        flowField.flowData[source] = packFlowFieldData(0, -1); // Direction index = 0, cost = -1
        std::cerr << "    inSrcPnt: " << source.first << "," << source.second << std::endl;
    }

   std::cerr << "  Process Q" << std::endl;
    while (!pq.empty()) {
        auto [currentCost, current] = pq.top();
        pq.pop();
        std::cerr << "  POINT: " << current.first <<"," << current.second << std::endl;

        if (costMap[current] < currentCost) {
        	std::cerr << "  => skip cost:" <<costMap[current] << " < " << currentCost << std::endl;
        	continue;
        }

        for (size_t i = 0; i < GridType::directions.size(); ++i) {
        	std::cerr << "  DIR: " << i << " " << GridType::directions[i].first << "," << GridType::directions[i].second << std::endl;
            Point neighbor = {current.first + GridType::directions[i].first, current.second + GridType::directions[i].second};
            // Wall around outside so can't go OOB
            if (grid[neighbor.second][neighbor.first] == GridToGraph::WALL) {
            	std::cerr << "  =>skip WALL" << std::endl;
                continue;
            }

            // Double the cost so can have 2 for horz, and 3 for diag
            int newCost = currentCost + 2 + (i>3 ? 1 : 0);
            if (newCost > maxCost) {
            	std::cerr << "  =>skip new cost: " << newCost << " > max: " << maxCost << std::endl;
            	continue;
            }

            if (costMap.find(neighbor) != costMap.end() && costMap[neighbor] <= newCost) {
            	std::cerr << "  =>skip: newCost larger: " << newCost << std::endl;
                continue;
            }

            costMap[neighbor] = newCost;
            pq.push({newCost, neighbor});

            // But use the actual number of moves for scaled cost
            // So 1.5 approximates the diagonal which is 1.412 (root 2)
            int scaledCost = calculateScaledCost(newCost/2, maxDistance);
            flowField.flowData[neighbor] = packFlowFieldData(i, scaledCost);
            std::cerr << "  =>Q ADD pnt: " << neighbor.first << "," << neighbor.second << " cost: " << newCost << " dir:" << i << " scaleCost:" << scaledCost << std::endl;
        }
    }
}

// Generate sparse flow field for an edge
SparseFlowField generateSparseFlowFieldForEdge(const GridType::Grid& grid,
												const GridType::Path& path,
												int maxCost,
												int maxDistance)
{
    SparseFlowField flowField;
    std::cerr << "----USE PATH: sz " << path.size() << " maxCost " << maxCost << " maxDist " << maxDistance << std::endl;

    // Dynamic sampling of key points along the path
    bool storeSample = true;
    std::vector<Point> sampledPath;
    sampledPath.push_back(path.front());
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Point prev = path[i - 1];
        Point current = path[i];
        Point next = path[i + 1];
    	std::cerr << "  cur: " << current.first << "," << current.second << " diff: " << current.first-prev.first << "," << current.second-prev.second
    			<< std::endl;

        int dx1 = current.first - prev.first;
        int dy1 = current.second - prev.second;
        int dx2 = next.first - current.first;
        int dy2 = next.second - current.second;

        if (dx1 != dx2 || dy1 != dy2) {
        	if (storeSample) {
        		sampledPath.push_back(current);
        		std::cerr << "      store: " << current.first << "," << current.second << std::endl;
        	}
        	storeSample = !storeSample;
        }
    }
    sampledPath.push_back(path.back());
    std::cerr << "  IN PATH: size: " << path.size() << " => sampled: " << sampledPath.size() << std::endl;

    bfsFlowField(grid, flowField, sampledPath, maxCost, maxDistance);
    return flowField;
}

// Parallelized flow field generation
void generateFlowFieldsParallel(const GridToGraph::Graph* graph, std::vector<SparseFlowField>& flowFields)
{
	const int maxDistance = std::max(graph->infoGrid.size()-3, graph->infoGrid[0].size()-3);
	const int maxCost = maxDistance /4;
    const auto& ablv = graph->abstractLevels[0];

	std::cerr << "############## MAKE PARALLEL: " << ablv.abstractEdges.size() << std::endl;
    std::vector<std::future<void>> futures;
    flowFields.resize(ablv.abstractEdges.size());
#if 0 // PARALLEL
    for (size_t i = 0; i < graph->abstractEdges.size(); ++i) {
    	std::cerr << "## EDGE: " << i << " path:" << graph->abstractEdges[i].path.size() << std::endl;
    	futures.push_back(std::async(std::launch::async, [graph, &flowFields, i, maxCost, maxDistance]() {
    		flowFields[i] = generateSparseFlowFieldForEdge(graph->infoGrid, graph->abstractEdges[i].path, maxCost, maxDistance);
    	}));

    }
    for (auto& future : futures) {
    	future.get();
    }
#else
    for (size_t i = 0; i < ablv.abstractEdges.size(); ++i) {
    	std::cerr << "## ABEDGE: " << i
    			<< " f:" << ablv.abstractEdges[i].from
				<< " (" << graph->baseNodes[ ablv.abstractNodes[ ablv.abstractEdges[i].from ].baseCenterNode ].first
				<< " ," << graph->baseNodes[ ablv.abstractNodes[ ablv.abstractEdges[i].from ].baseCenterNode ].second << ")"
    			<< " t:" << ablv.abstractEdges[i].to
				<< " (" << graph->baseNodes[ ablv.abstractNodes[ ablv.abstractEdges[i].to ].baseCenterNode ].first
				<< " ," << graph->baseNodes[ ablv.abstractNodes[ ablv.abstractEdges[i].to ].baseCenterNode ].second << ")"
				<< " pathSize:" << ablv.abstractEdges[i].path.size() << std::endl;
    	flowFields[i] = generateSparseFlowFieldForEdge(graph->infoGrid, ablv.abstractEdges[i].path, maxCost, maxDistance);
    }
#endif
}

}
