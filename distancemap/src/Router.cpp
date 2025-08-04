#include <algorithm>
#include "Router.hpp"

#include "MathUtils.h"

namespace Router {

    int pickNum(int range, int rnd)
{
    if (range <= 0) {
        return 0;
    }
    // Use modulo to wrap `rnd` into the range [0, range-1]
    return ((rnd % range) + range) % range;
}

float computeAngle(double dx, double dy) {
	if (dx == 0 && dy == 0) {
		std::cerr << "##NO MOVEMENT: dx=" << dx << " dy=" << dy << " dir ZERO" << std::endl;
		return 0.0; // No movement
	}
    return static_cast<float>(std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0));
}



GridType::Point nextPoint(const GridType::Point& from, const GridType::Point& dir) {
	std::cerr << "##NEXT POINT: From: " << from.first << "," << from.second
        << " move " << dir.first << "," << dir.second << std::endl;

	return { from.first + dir.first, from.second + dir.second };
}

int getNextEdge(const GridType::Point& pos, std::vector<GridType::Edge> edges,
    const GridType::Grid& infoGrid, const Routing::SparseGraph& routeGraph)
{
    std::cerr << "====GetNextEdge: " << pos.first << "," << pos.second << std::endl;
	int edgeIdx = routeGraph.edgeGrid.get(pos.first, pos.second) & 0xffff;
    int cell = infoGrid[pos.second][pos.first];
    std::cerr << "   => edgeGrid: " << edgeIdx << " infoGrid:" << std::hex << cell << std::dec << std::endl;
    if (cell & GridType::NODE) {
        int n = cell & 0xffff;
        const auto& connections = routeGraph.forward_adj[n];
        const int idx = pickNum(connections.size(), pos.first ^ pos.second);
        int edgeIdx = connections[idx].second;

        std::cerr << "      AT NODE. cons: " << connections.size()
    	<< " => idx:" << idx
    	<< " => edge: " << edgeIdx << std::endl;
        if (edges[edgeIdx].to == n) {
            std::cerr << " node:" << n << " is edge TO node => HALFWAY" << std::endl;
            edgeIdx |= GridType::EDGE_HALF;
        }
    }
    std::cerr << "   => " << (edgeIdx & GridType::EDGE_HALF ? "H " : "  ") << (edgeIdx & GridType::EDGE_MASK) << std::endl;
    return edgeIdx;
}

// Helper function to compute angle
GridType::Point nextStep(const GridType::Point& from, const GridType::Point& to) {
	std::cerr << "##NEXT STEP: From: " << from.first << "," << from.second
        << " -> " << to.first << "," << to.second << std::endl;

	GridType::Point dir = { to.first - from.first, to.second - from.second };
    return nextPoint(from, dir);
}

GridType::Point routeToNext(const GridToGraph::Graph& graph, const GridType::Point& source, const int srcEdgeIdx, const std::vector<int>& routeNodes)
{
    std::cerr << "## RouteToNext FIND ANGLE. edges:" << routeNodes.size() << std::endl;
    std::cerr << "ROUTE: ";
    for (int n : routeNodes)
    {
        std::cerr << " " << n;
    }
	std::cerr << std::endl;

	int srcCell = graph.infoGrid[source.second][source.first];
	std::cerr << "  Source: " << source.first << "," << source.second << std::hex << " => " << srcCell << std::dec << std::endl;
			

	bool incPath = false;
	const auto& fromto = graph.routingGraph.edgeFromTos[srcEdgeIdx];
	std::cerr << "Checking edge: " << fromto.first << "->" << fromto.second << std::endl;
	if (srcCell & GridType::XPND) {
		std::cerr << "  XPND" << std::endl;
		int dirIdx = GridType::get_XPND_DIR(srcCell);
		GridType::Point dir = GridType::directions8[dirIdx];
		std::cerr << "  XPND: Use idx: " << dirIdx << " for dir " << dir.first << "," << dir.second << std::endl;
		return nextPoint(source, dir);
	}
	else if (routeNodes.empty()) {
		std::cerr << "  EMPTY" << std::endl;
		std::cout << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
		std::cerr << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
	}
	else if (fromto.first == routeNodes.front()) {
		std::cerr << "  1ST" << std::endl;
		incPath = fromto.second == routeNodes[1];
		std::cerr << (incPath ? "increase " : "decrease") << " path dist to TOnode: " << fromto.second << std::endl;
	}
	else if (fromto.second == routeNodes.front()) {
		std::cerr << "  2ND" << std::endl;
		incPath = !fromto.first == routeNodes[1];
		std::cerr << (incPath ? "increase " : "decrease") << " path dist to FROMnode: " << fromto.first << std::endl;
	}
	else if (srcCell & GridType::EDGE) {
		std::cout << "ERROR: " << source.first << "," << source.second << " on edge, but not start of path" << std::endl;
		std::cerr << "ERROR: " << source.first << "," << source.second << " on edge, but not start of path" << std::endl;
		incPath = (srcCell & GridType::EDGE_HALF);
	}
	else
	{
		std::cout << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
		std::cerr << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
	}

    int edgeCell = graph.routingGraph.edgeGrid.get(source.first, source.second);
	std::cerr << "  EdgeCell: " << std::hex << edgeCell << std::dec << std::endl;
	int dist = edgeCell >> 16;
    // Check if on the edge
	if (dist == 0)
	{
		if (routeNodes.size() == 1)
		{
			std::cout << "ERROR: " << source.first << "," << source.second << " edgeCell "
				<< std::hex << edgeCell << std::dec << ", dist 0, but path = 1" << std::endl;
			std::cerr << "ERROR: " << source.first << "," << source.second << " edgeCell "
				<< std::hex << edgeCell << std::dec << ", dist 0, but path = 1" << std::endl;
            return source;
		}

        std::cerr << "  ON EDGE: Check next node " << routeNodes[1] << std::endl;
		for (const auto& idx : graph.routingGraph.nodeToEdgeIdxs[routeNodes[1]])
		{
			const auto& ft = graph.routingGraph.edgeFromTos[idx];
            const auto& path = graph.baseEdges[idx].path;
			std::cerr << "  Node: " << routeNodes[1] << " gave edge:" << idx
				<< " " << ft.first << "->" << ft.second << ". Check for that node" << std::endl;
			if (ft.first == routeNodes[1]) {
				std::cerr << "    => Found " << routeNodes[1] << " move forward" << std::endl;
                return nextStep(path.front(), path[1]);
			}
			else if (ft.second == routeNodes[1]) {
				std::cerr << "    => Found " << routeNodes[1] << " move backward" << std::endl;
                return nextStep(path.back(), path[path.size() - 2]);
            }
		}
        std::cout << "ERROR: " << source.first << "," << source.second << " edgeCell "
            << std::hex << edgeCell << std::dec << ". At node, but no edge going to " << routeNodes[1] << std::endl;
        std::cerr << "ERROR: " << source.first << "," << source.second << " edgeCell "
            << std::hex << edgeCell << std::dec << ". At node, but no edge going to " << routeNodes[1] << std::endl;
		return source;
	}
	const GridType::Edge& edge = graph.baseEdges[srcEdgeIdx];
	dist = std::clamp(dist + (incPath ? 1 : -1), 0, static_cast<int>(edge.path.size() - 1));
	auto movePnt = edge.path[dist];
	std::cerr << "  Use " << (incPath ? "increased" : "decrease") << " dist path[" << dist << "] ="
		<< movePnt.first << "," << movePnt.second << " move from: " << source.first<<","<<source.second << std::endl;

    return nextStep(source,movePnt);
}


GridType::Point getNextMove(const GridToGraph::Graph& graph, GridType::Point& source, GridType::Point& target)
{
    int srcCell = graph.infoGrid[source.second][source.first];
    if (srcCell & GridType::WALL)
    {
		if (srcCell & GridType::BOUNDARY) {
            int dirIdx = srcCell & GridType::DIR_MASK;
            source.first += GridType::directions8[dirIdx].first;
            source.second += GridType::directions8[dirIdx].second;
            std::cerr << source.first << "," << source.second << " DIR: " << dirIdx << std::endl;
        }

        if (graph.infoGrid[source.second][source.first] & GridType::WALL) {
            std::cout << "ERROR: " << source.first << "," << source.second << " is WALL: SRC ***************************" << std::endl;
            std::cerr << "ERROR: " << source.first << "," << source.second << " is WALL: SRC ***************************" << std::endl;
			return source;
        }
    }

    int tgtCell = graph.infoGrid[target.second][target.first];
    if (tgtCell & GridType::WALL)
    {
		if (tgtCell & GridType::BOUNDARY) {
            std::cerr << "MOVED TARGET " << target.first << "," << target.second << " => ";
            int dirIdx = tgtCell & GridType::DIR_MASK;
            target.first += GridType::directions8[dirIdx].first;
            target.second += GridType::directions8[dirIdx].second;
            std::cerr << target.first << "," << target.second << " DIR: " << dirIdx << std::endl;
        }

        if (graph.infoGrid[target.second][target.first] & GridType::WALL) {
            std::cout << "ERROR: " << target.first << "," << target.second << " is WALL: TARG ***************************" << std::endl;
            std::cerr << "ERROR: " << target.first << "," << target.second << " is WALL: TARG ***************************" << std::endl;
			return source;
        }
    }

    // Check all the levels and find the level at which the zones are adjacent
    // - Not sure if could do top down search, I *think* it's possible to be
    // (say) adjacent at level 2, but same at level 1, because zones are any shape
    int zoneSame = -1;
    int zoneAdjLevel = -1;
    for (int levelIdx=0; levelIdx < graph.abstractLevels.size(); ++levelIdx)
    {
        const auto& ablv = graph.abstractLevels[levelIdx];
        const auto& targetZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
        const auto& sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
        std::cerr << "LEVL: " << levelIdx << " src Zone(abNode): " << sourceZone << " dst Zone(abNode):" << targetZone << std::endl;
        // Check if in same zone for this level
        if (zoneSame == -1 && targetZone == sourceZone)
        {
            zoneSame = levelIdx;
            std::cerr << "==Zone Same: " << zoneSame << std::endl;
            continue;
        }

        // Check if target is one of the source neighbors
        for (int adjZone : ablv.zones[sourceZone].adjacentZones)
        {
            if (adjZone == targetZone)
            {
                zoneAdjLevel = levelIdx;
				std::cerr << "==ZoneLevel " << zoneAdjLevel << " target:" << targetZone << " adjacent to sourceZone : " << sourceZone << std::endl;
                break;
            }
        }
        // If found adjacent then we have the lowest level that they are adjacent
        if (zoneAdjLevel != -1)
        {
            break;
        }
    }

    // Check if found adjacent zones
    if (zoneAdjLevel != -1) {
        const auto& ablv = graph.abstractLevels[zoneAdjLevel];
        const auto& sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
        const auto& targetZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
        const auto& subgrid = ablv.subGrids[sourceZone];
		std::cerr << "==ZONE: " << sourceZone << " adjacent to: " << targetZone 
            << " map " << source.first << "," << source.second << " -"<<subgrid.offsetX<<","<<subgrid.offsetY << std::endl;
        uint16_t sub = subgrid.getCostFlow(source.first, source.second, subgrid.getFlow(targetZone));
        const auto& dir = GridType::directions8[sub & 0xf];
		std::cerr << "  Adjacent at Level: " << zoneAdjLevel << " = > srcZ:" << sourceZone << " tgtZ : " << targetZone << " subGrid : " << std::hex << sub << std::dec
			<< " => dir: " << dir.first << "," << dir.second << std::endl;
        return nextPoint(source, dir);

    }
    // No adjacent, check if found same zone
	if (zoneSame != -1) {
		const auto& infoGrid = graph.infoGrid;

        //
		// If on XPND then move towards edge
        //
		int srcCell = infoGrid[source.second][source.first];
		if (srcCell & GridType::XPND) {
			int dirIdx = GridType::get_XPND_DIR(srcCell);
			GridType::Point dir = GridType::directions8[dirIdx];
			std::cerr << "  SAME " << zoneSame << " ad XPND SRC = > xy:" << source.first << ", " << source.second
				<< " cell:" << std::hex << srcCell << std::dec << " dir:" << dirIdx
				<< " => " << dir.first << "," << dir.second << std::endl;
			return nextPoint(source, dir);
		}

		// If not on EDGE then move towards edge path
		// Get the edges to route from/to and get the route
		int sourceEdge = getNextEdge(source, graph.baseEdges, graph.infoGrid, graph.routingGraph);
		int targetEdge = getNextEdge(target, graph.baseEdges, graph.infoGrid, graph.routingGraph);
		std::cerr << "  SAME => Route: " << sourceEdge << " to " << targetEdge << std::endl;
        //
		// if both on the edge then move towards target
        //
		if ((sourceEdge & GridType::EDGE_MASK) == (targetEdge & GridType::EDGE_MASK))
		{
			std::cerr << "ROUTE: SAME EDGE: " << (sourceEdge & GridType::EDGE_MASK) << std::endl;
			const auto& sourcePath = graph.baseEdges[sourceEdge & GridType::EDGE_MASK].path;
			int sourceDist = graph.routingGraph.edgeGrid.get(source.first, source.second) >> 16;
			int targetDist = graph.routingGraph.edgeGrid.get(target.first, target.second) >> 16;
			std::cerr << "  => sourceDist: " << sourceDist << " targetDist: " << targetDist << std::endl;
			sourceDist += (sourceDist < targetDist) ? 1 : -1;
			std::cerr << "  => Path[" << sourceDist << "] = "
				<< sourcePath[sourceDist].first << "," << sourcePath[sourceDist].second << std::endl;
			return nextStep(source, sourcePath[sourceDist]);
		}

        //
		// Use the source and target edges to get a route
        //
		int srcEdgeIdx = (sourceEdge & GridType::EDGE_MASK);
		int dstEdgeIdx = (targetEdge & GridType::EDGE_MASK);
		const auto& ablv = graph.abstractLevels[zoneSame];
		int zoneIdx = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;

		std::cerr << "SAME ZONE. Diff EDGEs: zoneLevel: " << zoneSame << " zoneIndex: " << zoneIdx
			<< " srcEdge:" << srcEdgeIdx << " (" << graph.baseEdges[srcEdgeIdx].from << " -> " << graph.baseEdges[srcEdgeIdx].to << ")"
			<< " dstEdge:" << dstEdgeIdx << " (" << graph.baseEdges[dstEdgeIdx].from << " -> " << graph.baseEdges[dstEdgeIdx].to << ")"
			<< std::endl;

		const auto& sourceInfo = ablv.zones[zoneIdx];
		const auto& zoneBases = sourceInfo.baseNodeIdxs;
		const auto& zoneEdges = sourceInfo.baseEdgeIdxs;
		const auto routeNodes = Routing::findZonePath(graph.routingGraph, zoneBases, zoneEdges, srcEdgeIdx, dstEdgeIdx);
		return routeToNext(graph, source, srcEdgeIdx, routeNodes);
	}

    //
	// Use last Abstract Level to route.
    //
	std::cerr << "  Not Same or Adjacent. abstract base/edges" << std::endl;
	int srcEdgeIdx = getNextEdge(source, graph.baseEdges, graph.infoGrid, graph.routingGraph) & GridType::EDGE_MASK;
	int dstEdgeIdx = getNextEdge(target, graph.baseEdges, graph.infoGrid, graph.routingGraph) & GridType::EDGE_MASK;
	const auto routeNodes = Routing::findZonePath(graph.routingGraph,
		graph.routingGraph.abstractBaseNodes, graph.routingGraph.abstractBaseEdges, srcEdgeIdx, dstEdgeIdx);
	return routeToNext(graph, source, srcEdgeIdx, routeNodes);
}
	

//////////////////////////////////////////////////////////////////////////////

float getAngle(const GridToGraph::Graph& graph, const Router::Info& info,
					RouteCtx* ctx, godot::Vector2 from, godot::Vector2 to, int type) {
		GridType::Point fromPnt = { from.x / (info.mCellWidth * 8), from.y / (info.mCellHeight * 8) };
		GridType::Point toPnt = { to.x / (info.mCellWidth * 8), to.y / (info.mCellHeight * 8) };
		std::cerr << "===GETMOVE: " << from.x << "," << to.x << "  cell: " << info.mCellWidth << "x" << info.mCellHeight << " => " << fromPnt.first << "," << fromPnt.second
			<< " to:" << toPnt.first << "," << toPnt.second << std::endl;

		std::cerr << "********** FROM:" << from.x << "," << from.y << " TO " << to.x << "," << to.y
			<< "    " << fromPnt.first << "," << fromPnt.second << " TO " << toPnt.first << "," << toPnt.second << std::endl;
		std::cerr << "CTX: F: " << ctx->from.first << "," << ctx->from.second
			<< " C T: " << ctx->to.first << "," << ctx->to.second
			<< " C N: " << ctx->next.first << "," << ctx->next.second
			<< " C DIR: " << ctx->curDir
			<< " FRM " << from.x << "," << from.y
			<< " TO  " << to.x << "," << to.y
			<< " FPNT: " << fromPnt.first << "," << fromPnt.second
			<< " TPNT: " << toPnt.first << "," << toPnt.second
			<< std::endl;

		if (ctx->type == type) {
			if (fromPnt.first != ctx->next.first || fromPnt.second != ctx->next.second) {
				std::cerr << "===REUSED DIR " << ctx->curDir
					<< " frm: " << fromPnt.first << "," << fromPnt.second
					<< " nxt: " << ctx->next.first << "," << ctx->next.second << std::endl;
				return ctx->curDir;
			}
		}

		ctx->from = fromPnt;
		ctx->to = toPnt;
		ctx->type = type;

		std::cerr << "===GETMOVE: " << fromPnt.first << "," << fromPnt.second << " TO " << toPnt.first << "," << toPnt.second << std::endl;
		ctx->next = getNextMove(graph, fromPnt, toPnt);
		ctx->curDir = computeAngle(ctx->next.first - ctx->from.first, ctx->next.second - ctx->from.second);
		std::cerr << "##ANG from:" << fromPnt.first << "," << fromPnt.second << " NXT: " << ctx->next.first << "," << ctx->next.second << " = " << ctx->curDir << std::endl;
		for (int ii = 0; ii < GridType::directions8.size(); ++ii) {
			int xx = GridType::directions8[ii].first;
			int yy = GridType::directions8[ii].second;
			std::cerr << "                  XXX dir: " << ii << " => " << xx << "," << yy << " => " << computeAngle(xx, yy) << std::endl;
		}
		return ctx->curDir;
	}
}

