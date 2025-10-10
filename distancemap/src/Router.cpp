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
		return 0.0; // No movement
	}
	return static_cast<float>(std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0));
}

int getClosestNode(const GridType::Point& pos,
		const GridType::Grid& infoGrid, const Routing::SparseGraph& routingGraph)
{
  int edgeIdx = routingGraph.edgeGrid.get(pos.first, pos.second) & GridType::EDGE_MASK;
  int cell = infoGrid[pos.second][pos.first];
  std::cerr << "getClosest: " << pos.first <<"," << pos.second << " => edgeIdx: " << edgeIdx
    << " cell: " << std::hex << cell << std::dec << std::endl;
  if (cell & GridType::NODE) {
    std::cerr << "getClosest: at NODE " << (cell&0xffff) << std::endl;
    return cell & 0xffff;
  }

  if (cell & GridType::EDGE) {
    int edgeIdx = cell & GridType::EDGE_MASK;
    const auto& fromto = routingGraph.edgeFromTos[edgeIdx];
    std::cerr << "getClosest: edge: " << edgeIdx << " fromto: " << fromto.first <<"->" << fromto.second << std::endl;
    return (cell&GridType::EDGE_HALF) ? fromto.second : fromto.first;
  }
  if (cell & GridType::XPND) {
    int dst = GridType::get_XPND_DIST(cell);
    int dir = GridType::get_XPND_DIR(cell);
    auto edgePnt = pos;
    edgePnt.first += GridType::directions8[dir].first * dst;
    edgePnt.second += GridType::directions8[dir].second * dst;
    cell = infoGrid[edgePnt.second][edgePnt.first];
    std::cerr << "GNE: XPND => move " << pos.first <<","<< pos.second
      << " in dir:" << dir << " by dst:" << dst
      << " => " << edgePnt.first <<","<< edgePnt.second
      << " cell: " << std::hex << cell << std::dec << std::endl;
    if (cell & GridType::EDGE) {
      std::cerr << "getClosest: At EDGE => " << (cell&GridType::EDGE_MASK) << std::endl;
      int edgeIdx = cell & GridType::EDGE_MASK;
      const auto fromto = routingGraph.edgeFromTos[edgeIdx];
      std::cerr << "  => node: " << ((cell&GridType::EDGE_HALF) ? fromto.second : fromto.first) << std::endl;
      return (cell&GridType::EDGE_HALF) ? fromto.second : fromto.first;
    }
    else if (cell & GridType::NODE) {
      int node = cell & 0xffff;
      std::cerr << "getClosest: Moved to NODE " << node << std::endl;
      return routingGraph.forward_adj[node][0].second;
    }
    else
    {
      std::cerr << "ERROR: Cell not moved to edge. dir: " << dir << " dst: " << dst
        << " (" << edgePnt.first <<","<< edgePnt.second << ") => " << std::hex << cell << std::dec << std::endl;
      return -1;
    }
  }
  std::cerr << "ERROR: Cell not XPND or EDGE or NODEL:  "
    << pos.first <<","<< pos.second << " => " << std::hex << cell << std::dec << std::endl;
	return -1;
}

GridType::Point nextPoint(const GridType::Point& from, const GridType::Point& dir) {
	return { from.first + dir.first, from.second + dir.second };
}

// Helper function to compute angle
GridType::Point nextStep(const GridType::Point& from, const GridType::Point& to) {
	GridType::Point dir = { to.first - from.first, to.second - from.second };
	//xxx return nextPoint(from, dir);
	GridType::Point p = nextPoint(from, dir);
	std::cerr << "  NXTSTP f: " << from.first <<","<< from.second << " t: " << to.first <<","<< to.second
		<< " => d:" << dir.first <<","<< dir.second
		<< " => p: " << p.first <<","<< p.second << std::endl;
	return p;
}

GridType::Point getNextMove(const GridToGraph::Graph& graph, RouteCtx* ctx, GridType::Point source, GridType::Point target)
{
  // Check if source in a wall
  int srcCell = graph.infoGrid[source.second][source.first];
  std::cerr << "GetNextMove: " << source.first <<","<< source.first << " src: " << std::hex << srcCell << std::dec << std::endl;
  if (srcCell & GridType::WALL)
  {
    // If boundary then use dir to move ouit of wall
    if (srcCell & GridType::BOUNDARY) {
      int dirIdx = srcCell & GridType::DIR_MASK;
      source.first += GridType::directions8[dirIdx].first;
      source.second += GridType::directions8[dirIdx].second;
      std::cerr <<"GM: SRC BOUNDARY => src: " << source.first <<","<< source.second << std::endl;
    }

    // If still in wall then error
    srcCell = graph.infoGrid[source.second][source.first];
    if (srcCell & GridType::WALL) {
      std::cout << "ERROR: " << source.first << "," << source.second << " is WALL: SRC ***************************" << std::endl;
    }
    return source;
  }

  // Check target in wall
  int tgtCell = graph.infoGrid[target.second][target.first];
  if (tgtCell & GridType::WALL)
  {
    // If boundary then use dir to move ouit of wall
    if (tgtCell & GridType::BOUNDARY) {
      int dirIdx = tgtCell & GridType::DIR_MASK;
      target.first += GridType::directions8[dirIdx].first;
      target.second += GridType::directions8[dirIdx].second;
      std::cerr <<"GM: TGT BOUNDARY => tgt: " << target.first <<","<< target.second << std::endl;
    }

    // If still in wall then error
    if (graph.infoGrid[target.second][target.first] & GridType::WALL) {
      std::cout << "ERROR: " << target.first << "," << target.second << " is WALL: TARG ***************************" << std::endl;
      return source;
    }
  }

  //
  // If on XPND then move towards edge
  //
  if (srcCell & GridType::XPND)
  {
    int dirIdx = GridType::get_XPND_DIR(srcCell);
    GridType::Point dir = GridType::directions8[dirIdx];
    std::cerr << "GM: At XPND move in dir: " << dir.first<<","<<dir.second << std::endl;
    return nextPoint(source, dir);
  }

  // Check all the levels and find the level at which the zones are adjacent
  // - Not sure if could do top down search, I *think* it's possible to be
  // (say) adjacent at level 2, but same at level 1, because zones are any shape
  // - Also track the lowest (smallest) zone index which the source and target are
  // both in in case we don't get adjacent
  int ablvIdx= -1;
  int targetZone = -1;
  int sourceZone = -1;
  for (int levelIdx=0; (ablvIdx == -1) &&(levelIdx < graph.abstractLevels.size()); ++levelIdx)
  {
    const auto& ablv = graph.abstractLevels[levelIdx];
    targetZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
    sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
    // Check if in same zone for this level
    if (targetZone == sourceZone)
    {
      ablvIdx = levelIdx;
      break;
    }

    // Check if target is one of the source neighbors
    for (int adjZone : ablv.zones[sourceZone].adjacentZones)
    {
      if (adjZone == targetZone)
      {
        ablvIdx = levelIdx;
        break;
      }
    }
  }
  std::cerr << "GM: Found: srcZ: " << sourceZone << " tgtZ: " << targetZone << " idx: " << ablvIdx << std::endl;

  // Get the ablv. Use top level if more than 1 zone apart
  const auto& ablv = (ablvIdx != -1) 
    ? graph.abstractLevels[ablvIdx]
    : graph.abstractLevels[graph.abstractLevels.size()-1];

  // Route through the appropriate level based on zone distance
  // If at Node use that
  std::optional<int> srcNodeIdx = (srcCell & GridType::NODE)
    ? std::optional<int>(srcCell & 0xffff)
    : std::nullopt;
  // If at Edge use that
  std::optional<int> srcEdgeIdx = (srcCell & GridType::EDGE)
    ? std::optional<int>(srcCell & GridType::EDGE_MASK)
    : std::nullopt;
  // Otherwise find the closest node
  if (! srcNodeIdx.has_value() && ! srcEdgeIdx.has_value()) {
    std::cerr << "GM: Not N/E FindCloset to source " << source.first <<"," << source.second << std::endl;
    srcNodeIdx = getClosestNode(source, graph.infoGrid, graph.routingGraph);
  }

  std::cerr << "GM: source node/edge found. Find target" << std::endl;

  // If same zone then use the node 
  // want to have "find closest node using XPND edges NODE"
  int tgtNodeIdx = (sourceZone == targetZone) 
    ? getClosestNode(target, graph.infoGrid, graph.routingGraph)
    : ablv.abstractNodes[targetZone].baseCenterNode;

  // -1 = same zone, >=0 = adjacent zone index, -2 = distant
  int zoneRelationship = (ablvIdx == -1) ? -2
    : (sourceZone == targetZone) ? -1 : ablvIdx;

  std::cerr << "GM: tgtNode: " << tgtNodeIdx << " => Find routeNodes" << std::endl;
  ctx->routeNodes = Routing::findRoute(
      graph.routingGraph,
      ctx,
      ablv.zones,
      ablv.abstractEdges,
      srcNodeIdx,
      srcEdgeIdx,
      tgtNodeIdx,
      sourceZone,
      targetZone,
      zoneRelationship);

  // Route using the list of nodes for the path
  std::cerr << "GM: Use routeNodes: ";
  for (int n : ctx->routeNodes) {
    std::cerr << n << " ";
  }
  std::cerr << std::endl;
  if (ctx->routeNodes.empty()) {
    std::cerr << "ERROR: No routeNodes" << std::endl;
    return nextStep(source, target);
  }

  // If at a Node then search edges to find one going from node
  if (srcCell & GridType::NODE) {
    int srcNode = srcCell & 0xffff;
    std::cerr << "GM: src at Node: " << srcNode << std::endl;
    int toIdx = (srcNode == ctx->routeNodes[0])
      ? ctx->routeNodes[1]
      : ctx->routeNodes[0];
    std::cerr << "GM: src: " << srcNode << " to:" << toIdx << std::endl;

    // Find edge connecting nodeA to nodeB
    for (const auto& [neighbor, edgeIdx] : graph.routingGraph.forward_adj[srcNode]) {
      std::cerr << "GM:   Check edge:" << edgeIdx << " neigh: " << neighbor << " vs to: " << toIdx << std::endl;
      if (neighbor == toIdx) {
        const auto& edge = graph.baseEdges[edgeIdx];
        const auto& nxt = (edge.from == srcNode) ? edge.path[1] : edge.path[edge.path.size()-2];
        std::cerr << "GM:  edge: "<< edgeIdx << " " << edge.from <<"->"<< edge.to
          << " nxtPnt:" << nxt.first<<","<<nxt.second << std::endl;
        return nextStep(source, nxt);
      }
    }
    std::cerr << "ERROR: Failed to find edge for node:" << srcNode << "->" << toIdx << std::endl;
  }

  // If at Edge then find the source on the path and move to the next/prev
  if (srcCell & GridType::EDGE) {
    int edgeIdx = srcCell & GridType::EDGE_MASK;
    std::cerr << "GM: src at Edge: " << edgeIdx << std::endl;
    const auto& edge = graph.baseEdges[edgeIdx];

    int index = std::find(edge.path.begin(), edge.path.end(), source) - edge.path.begin();
    if (index != edge.path.size()) {
    std::cerr << "GM:   found src: " << source.first<<","<<source.second << " in "
      << edge.from <<"->"<< edge.to << " path sz:" << edge.path.size() << " at idx: " << index
      << std::endl;
      GridType::Point nxt;
      index += (edge.from == ctx->routeNodes[0]) ? -1 : 1;
      if (index < 0) {
        nxt = graph.baseNodes[edge.from];
        std::cerr << "GM:  At path start use fromNode: " << edge.from;
      }
      else if (index >= edge.path.size()) {
        nxt = edge.toDeadEnd 
          ? graph.deadEnds[edge.to]
          : graph.baseNodes[edge.to];
        std::cerr << "GM:  At path end use toNode: " << edge.to;
      }
      else {
        nxt = edge.path[index];
        std::cerr << "GM:  move along path to idx:" << index;
      }
      std::cerr << " nxtPnt:" << nxt.first<<","<<nxt.second << std::endl;
      return nextStep(source, nxt);
    }
    std::cerr << "ERROR: Failed to src: " << source.first<<","<<source.second
      << " in path for edge: " << edgeIdx << std::endl;
    return nextStep(source, target);
  }
  std::cerr << "ERROR: Not on EDGE or NODE" << std::endl;
  return nextStep(source, target);
}


//////////////////////////////////////////////////////////////////////////////

float getAngle(const GridToGraph::Graph& graph, const Router::Info& info,
		RouteCtx* ctx, godot::Vector2 from, godot::Vector2 to, int type) {
	GridType::Point fromPnt = { from.x / (info.mCellWidth * 8), from.y / (info.mCellHeight * 8) };
	GridType::Point toPnt = { to.x / (info.mCellWidth * 8), to.y / (info.mCellHeight * 8) };
	std::cerr << "===GETMOVE: " << from.x << "," << to.x << "  cell: " << info.mCellWidth << "x" << info.mCellHeight << " => " << fromPnt.first << "," << fromPnt.second
		<< " to:" << toPnt.first << "," << toPnt.second << std::endl;

	std::cerr << "======FROM:" << from.x << "," << from.y << " TO " << to.x << "," << to.y
		<< "    " << fromPnt.first << "," << fromPnt.second << " TO " << toPnt.first << "," << toPnt.second << std::endl;
	std::cerr << "CTX: F: " << ctx->from.first << "," << ctx->from.second
		<< " CtxTo: " << ctx->to.first << "," << ctx->to.second
		<< " CtxNxt: " << ctx->next.first << "," << ctx->next.second
		<< " CtxtDir: " << ctx->curDir
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
	ctx->next = getNextMove(graph, ctx, fromPnt, toPnt);
	ctx->curDir = computeAngle(ctx->next.first - ctx->from.first, ctx->next.second - ctx->from.second);
	std::cerr << "==>ANG from:" << fromPnt.first << "," << fromPnt.second << " NXT: " << ctx->next.first << "," << ctx->next.second << " = " << ctx->curDir << std::endl;
	return ctx->curDir;
}

} //namespace

