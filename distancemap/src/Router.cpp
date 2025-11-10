#include <tuple>
#include <algorithm>
#include "Router.hpp"

#include "MathUtils.h"

#include "Debug.h"

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

// Returns { closestNode, [edgeIdx | deadNode | -1], isEdgeIdx }
//
std::tuple<int, int, bool> getClosestNode(const GridType::Point& pos,
	const GridType::Grid& infoGrid, const Routing::SparseGraph& routingGraph)
{
	const int edgeIdx = routingGraph.edgeGrid.get(pos.first, pos.second) & GridType::EDGE_MASK;
	int cell = infoGrid[pos.second][pos.first];
    LOG_DEBUG("getClosest: " << pos.first << "," << pos.second << " => edgeIdx: " << edgeIdx
        << " (" << (routingGraph.edgeFromTos[edgeIdx].first) << "->"
        << (routingGraph.edgeFromTos[edgeIdx].second) << ")"
        << " cell: " << std::hex << cell << std::dec);
	if (cell & GridType::NODE) {
        LOG_DEBUG("getClosest: at NODE " << (cell & 0xffff));
        return { cell & 0xffff, -1, false};
	}

	if (cell & GridType::EDGE) {
		int edgeIdx = cell & GridType::EDGE_MASK;
		const auto& fromto = routingGraph.edgeFromTos[edgeIdx];
        LOG_DEBUG("getClosest: edge: " << edgeIdx << " fromto: " << fromto.first << "->" << fromto.second);
        if (!(cell & GridType::EDGE_HALF) || fromto.second < 0) {
            return { fromto.first, edgeIdx, true };
        }
        else {
            return { fromto.second, edgeIdx, true };
        }
	}
	GridType::Point edgePnt = pos;
	while (cell & GridType::XPND) {
		int dst = GridType::get_XPND_DIST(cell);
		int dir = GridType::get_XPND_DIR(cell);
		edgePnt.first += GridType::directions8[dir].first * dst;
		edgePnt.second += GridType::directions8[dir].second * dst;
		cell = infoGrid[edgePnt.second][edgePnt.first];
        LOG_DEBUG("GNE: XPND => move in dir:" << dir << " by dst:" << dst
            << " => " << edgePnt.first << "," << edgePnt.second
            << " cell: " << std::hex << cell << std::dec);
	}
	if (cell & GridType::EDGE) {
        LOG_DEBUG("getClosest: At EDGE => " << (cell & GridType::EDGE_MASK));
		int edgeIdx = cell & GridType::EDGE_MASK;
		const auto fromto = routingGraph.edgeFromTos[edgeIdx];
        LOG_DEBUG("  => node: " << ((cell & GridType::EDGE_HALF) ? fromto.second : fromto.first));
        if (!(cell & GridType::EDGE_HALF) || fromto.second < 0) {
            return { fromto.first, edgeIdx, true };
        }
        else {
            return { fromto.second, edgeIdx, true };
        }
	}
	if (cell & GridType::NODE) {
		int node = cell & 0xffff;
        LOG_DEBUG("getClosest: Moved to NODE " << node);
        return { node, -1, false };
	}
	if (cell & GridType::DEND) {
		int toDend = cell & 0xffff;
		int from = routingGraph.edgeFromTos[edgeIdx].first;
        LOG_DEBUG("getClosest: Moved to DEND " << toDend << " fromNode: " << from);
        return { from, toDend, false };
	}
	else
	{
        LOG_ERROR("Cell not moved to edge "
            << " (" << edgePnt.first << "," << edgePnt.second << ") => " << std::hex << cell << std::dec);
        return { -1, -1, false };
	}
}

GridType::Point nextPoint(const GridType::Point& from, const GridType::Point& dir) {
	return { from.first + dir.first, from.second + dir.second };
}

// Helper function to compute angle
GridType::Point nextStep(const GridType::Point& from, const GridType::Point& to) {
	GridType::Point dir = { to.first - from.first, to.second - from.second };
	//xxx return nextPoint(from, dir);
	GridType::Point p = nextPoint(from, dir);
    LOG_DEBUG("  NXTSTP f: " << from.first << "," << from.second << " t: " << to.first << "," << to.second
        << " => d:" << dir.first << "," << dir.second
        << " => p: " << p.first << "," << p.second);
	return p;
}

GridType::Point getNextMove(const GridToGraph::Graph& graph, RouteCtx* ctx, GridType::Point source, GridType::Point target)
{
  // Check if source in a wall
  int srcCell = graph.infoGrid[source.second][source.first];
  LOG_DEBUG("GetNextMove: " << source.first << "," << source.first << " src: " << std::hex << srcCell << std::dec);
  if (srcCell & GridType::WALL)
  {
    // If boundary then use dir to move ouit of wall
    if (srcCell & GridType::BOUNDARY) {
      int dirIdx = srcCell & GridType::DIR_MASK;
      source.first += GridType::directions8[dirIdx].first;
      source.second += GridType::directions8[dirIdx].second;
      LOG_DEBUG("GM: SRC BOUNDARY => src: " << source.first << "," << source.second);
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
      LOG_DEBUG("GM: TGT BOUNDARY => tgt: " << target.first << "," << target.second);
    }

    // If still in wall then error
	if (graph.infoGrid[target.second][target.first] & GridType::WALL) {
		LOG_ERROR(target.first << "," << target.second << " is WALL: TARG ***************************");
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
    LOG_DEBUG("GM: At XPND move in dir: " << dir.first << "," << dir.second);
    return nextPoint(source, dir);
  }

  if (!ctx->routeNodes.empty()) {
      if (srcCell & GridType::NODE) {
          int from = srcCell & 0xffff;
          LOG_DEBUG("DBG: At node: " << from);
          if (ctx->routeNodes[0] == from) {
              LOG_DEBUG("DBG: which is 1st of routeNodes");
              ctx->routeNodes.erase(ctx->routeNodes.begin());
              if (!ctx->routeNodes.empty()) {
                  int to = ctx->routeNodes[0];
                  int edgeIdx = graph.routingGraph.getEdgeFromNodes(from, to);
                  const auto pnt = graph.baseEdges[edgeIdx].path[1];
                  LOG_DEBUG("DBG: N1: " << from << " N2:" << to << " E: " << edgeIdx
                      << " return path[1] " << pnt.first << "," << pnt.second);
                  return pnt;
              }
          } 
      }
      
	  if (srcCell & GridType::EDGE)
	  {
		  int edgeIdx = srcCell & GridType::EDGE_MASK;
          const auto& edge = graph.baseEdges[edgeIdx];
          LOG_DEBUG("DBG: At EDGE " << edgeIdx << " (" << edge.from << "->" << edge.to << ") => follow ");
		  auto it = std::find(edge.path.begin(), edge.path.end(), source);
		  if (it != edge.path.end()) {
			  size_t index = std::distance(edge.path.begin(), it);
			  int adj = (edge.from == ctx->routeNodes[0]) ? 1
				  : (edge.to == ctx->routeNodes[0])
				  ? -1
				  : 0;
              LOG_DEBUG("DBG: Found src in path index:" << index << " adj: " << adj
                  << " move to " << edge.path[index + adj].first << "," << edge.path[index + adj].second);
			  return edge.path[index + adj];
		  }
		  else {
			  std::cout << "DBG: Source not found in path" << std::endl;
		  }
	  }
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
  LOG_DEBUG("GM: Found: srcZ: " << sourceZone << " tgtZ: " << targetZone << " idx: " << ablvIdx);

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
  if (! srcNodeIdx && ! srcEdgeIdx) {
      LOG_DEBUG("GM: Not N/E FindCloset to source " << source.first << "," << source.second);
    const auto& srcEdgeOrDed = getClosestNode(source, graph.infoGrid, graph.routingGraph);
    srcNodeIdx = std::get<0>(srcEdgeOrDed);
  }

  LOG_DEBUG("GM: source node/edge found. Find target");

  // If same zone then use the node 
  // want to have "find closest node using XPND edges NODE"
  std::tuple<int, int, bool> tgtEdgeOrDed = (sourceZone == targetZone)
      ? getClosestNode(target, graph.infoGrid, graph.routingGraph)
      : std::make_tuple(ablv.abstractNodes[targetZone].baseCenterNode, -1, false);

  int tgtNodeIdx = std::get<0>(tgtEdgeOrDed);
  // -1 = same zone, >=0 = adjacent zone index, -2 = distant
  int zoneRelationship = (ablvIdx == -1) ? -2
	  : (sourceZone == targetZone) ? -1
      : ablvIdx;

  // If in the same zone, the src and tgt are the same node
  // then use the edge connecting to the other node
  if (zoneRelationship == -1
      && srcNodeIdx && *srcNodeIdx == tgtNodeIdx
      && std::get<1>(tgtEdgeOrDed) >= 0)
  {
      int edgeIdx = (std::get<2>(tgtEdgeOrDed)) 
          ? std::get<1>(tgtEdgeOrDed)
          : graph.routingGraph.getEdgeFromDead(std::get<1>(tgtEdgeOrDed));
      if (edgeIdx != -1) {
          const auto& edge = graph.baseEdges[edgeIdx];
          const auto& path = edge.path;
          const auto it = std::find(path.begin(), path.end(), source);
          if (it != path.end()) {
			  int idx = std::distance(path.begin(), it);
              LOG_DEBUG("DBG=== Found Source " << source.first << "," << source.second
                  << " idx: " << idx << " tgeNode: " << tgtNodeIdx
                  << " (" << edge.from << "->" << edge.to << (edge.toDeadEnd ? " (DED)" : ""));
              if (tgtNodeIdx != edge.from) {
                  idx = std::max(0, idx - 1);
                  LOG_DEBUG("DBG=== Move BCK to " << path[idx].first << "," << path[idx].second);
                  return nextStep(source, path[idx]);
              }
              else {
                  idx = std::min(static_cast<int>(path.size())-1, idx + 1);
                  LOG_DEBUG("DBG=== Move FWD to " << path[idx].first << "," << path[idx].second);
                  return nextStep(source, path[idx]);
              }
          }
          else {
              LOG_DEBUG("DBG=== Source " << source.first << "," << source.second
                  << " Not found in Edge: " << edgeIdx
                  << " (" << edge.from << "->" << edge.to << (edge.toDeadEnd ? " (DED)" : ""));
          }
      }
      else {
          LOG_DEBUG("DBG=== No edge found");
      }
  }
  LOG_DEBUG("GM: tgtNode: " << tgtNodeIdx << " => Find routeNodes");
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
  LOG_DEBUG_CONT("GM: Use routeNodes: ");
  LOG_DEBUG_FOR(int n : ctx->routeNodes, n << " ");
  if (ctx->routeNodes.empty()) {
    if (zoneRelationship == -1) {
        LOG_DEBUG("GM: No routeNodes, but same zone => use tgtNode: " << tgtNodeIdx);
        ctx->routeNodes.push_back(tgtNodeIdx);
    }
    else {
        LOG_DEBUG("ERROR: No routeNodes srcZ:" << sourceZone << " tgtZ:" << targetZone);
        return nextStep(source, target);
    }
  }

  // If at a Node then search edges to find one going from node
  if (srcCell & GridType::NODE) {
    int srcNode = srcCell & 0xffff;
    LOG_DEBUG("GM: src at Node: " << srcNode);
    int toIdx = (srcNode == ctx->routeNodes[0])
      ? ctx->routeNodes[1]
      : ctx->routeNodes[0];
    LOG_DEBUG("GM: src: " << srcNode << " to:" << toIdx);

    // Find edge connecting nodeA to nodeB
    for (const auto& [neighbor, edgeIdx] : graph.routingGraph.forwardConnections[srcNode]) {
        LOG_DEBUG("GM:   Check edge:" << edgeIdx << " neigh: " << neighbor << " vs to: " << toIdx);
      if (neighbor == toIdx) {
        const auto& edge = graph.baseEdges[edgeIdx];
        const auto& nxt = (edge.from == srcNode) ? edge.path[1] : edge.path[edge.path.size()-2];
        LOG_DEBUG("GM:  edge: " << edgeIdx << " " << edge.from << "->" << edge.to
            << " nxtPnt:" << nxt.first << "," << nxt.second);
        return nextStep(source, nxt);
      }
    }
    LOG_DEBUG("ERROR: Failed to find edge for node:" << srcNode << "->" << toIdx);
  }

  // If at Edge then find the source on the path and move to the next/prev
  if (srcCell & GridType::EDGE) {
    int edgeIdx = srcCell & GridType::EDGE_MASK;
    LOG_DEBUG("GM: src at Edge: " << edgeIdx);
    const auto& edge = graph.baseEdges[edgeIdx];

    int index = std::find(edge.path.begin(), edge.path.end(), source) - edge.path.begin();
    if (index != edge.path.size()) {
        LOG_DEBUG("GM:   found src: " << source.first << "," << source.second << " in "
            << edge.from << "->" << edge.to << " path sz:" << edge.path.size() << " at idx: " << index);
      GridType::Point nxt;
      index += (edge.from == ctx->routeNodes[0]) ? -1 : 1;
      if (index < 0) {
        nxt = graph.baseNodes[edge.from];
        LOG_DEBUG_CONT("GM:  At path start use fromNode: " << edge.from);
      }
      else if (index >= edge.path.size()) {
        nxt = edge.toDeadEnd 
          ? graph.deadEnds[edge.to]
          : graph.baseNodes[edge.to];
        LOG_DEBUG_CONT("GM:  At path end use toNode: " << edge.to);
      }
      else {
        nxt = edge.path[index];
        LOG_DEBUG_CONT("GM:  move along path to idx:" << index);
      }
      LOG_DEBUG(" nxtPnt:" << nxt.first << "," << nxt.second);
      return nextStep(source, nxt);
    }
    LOG_ERROR("Failed to src: " << source.first << "," << source.second
        << " in path for edge: " << edgeIdx);
    return nextStep(source, target);
  }
  LOG_ERROR("Not on EDGE or NODE");
  return nextStep(source, target);
}


//////////////////////////////////////////////////////////////////////////////

float getAngle(const GridToGraph::Graph& graph, const Router::Info& info,
		RouteCtx* ctx, godot::Vector2 from, godot::Vector2 to, int type) {
	GridType::Point fromPnt = { from.x / (info.mCellWidth * 8), from.y / (info.mCellHeight * 8) };
	GridType::Point toPnt = { to.x / (info.mCellWidth * 8), to.y / (info.mCellHeight * 8) };
    LOG_DEBUG("---GETANGLE: " << from.x << "," << to.x << "  cell: " << info.mCellWidth << "x" << info.mCellHeight << " => " << fromPnt.first << "," << fromPnt.second
        << " to:" << toPnt.first << "," << toPnt.second);

    LOG_DEBUG("------FROM:" << from.x << "," << from.y << " TO " << to.x << "," << to.y
        << "    " << fromPnt.first << "," << fromPnt.second << " TO " << toPnt.first << "," << toPnt.second);
    LOG_DEBUG("      CTX: F: " << ctx->from.first << "," << ctx->from.second
        << " CtxTo: " << ctx->to.first << "," << ctx->to.second
        << " CtxNxt: " << ctx->next.first << "," << ctx->next.second
        << " CtxtDir: " << ctx->curDir
        << " FPNT: " << fromPnt.first << "," << fromPnt.second
        << " TPNT: " << toPnt.first << "," << toPnt.second
        << " last: " << ((int)ctx->lastRouteType) << " ctxType: " << ctx->type << " type: " << type);

	if (ctx->type == type) {
        auto dx = ctx->next.first - fromPnt.first;
        auto dy = ctx->next.second - fromPnt.second;
		if (dx || dy) {
            bool allowReuse = (ctx->reuseInit) ? --ctx->reuseCnt : true;
            if (allowReuse) {
                LOG_DEBUG_CONT("---REUSE ("<<ctx->reuseCnt<< ") PNT(dxdy: " << dx << ", " << dy << ") "
                    << " frm: " << fromPnt.first << "," << fromPnt.second
                    << " nxt: " << ctx->next.first << "," << ctx->next.second);
                ctx->didReuse = true;
                auto nxtDir = computeAngle(dx, dy);
                if (nxtDir != ctx->curDir) {
                    LOG_DEBUG("  CHANGE dir " << ctx->curDir << " => " << nxtDir);
                    ctx->curDir = nxtDir;
                }
                else {
                    LOG_DEBUG("  SAME dir " << ctx->curDir);
                }
                return ctx->curDir;
            }
            else {
                LOG_DEBUG("---STOP: REUSE");
				ctx->reuseCnt = ctx->reuseInit;
            }
		}
	}

	ctx->from = fromPnt;
	ctx->to = toPnt;
	ctx->type = type;

	LOG_DEBUG("===GETNEXTMOVE: " << fromPnt.first << "," << fromPnt.second << " TO " << toPnt.first << "," << toPnt.second
		<< " ruse:" << ctx->reuseCnt);
	ctx->next = getNextMove(graph, ctx, fromPnt, toPnt);
	ctx->curDir = computeAngle(ctx->next.first - ctx->from.first, ctx->next.second - ctx->from.second);
    LOG_DEBUG("==>ANG from:" << fromPnt.first << "," << fromPnt.second
        << " NXT: " << ctx->next.first << "," << ctx->next.second<< " = " << ctx->curDir);
	return ctx->curDir;
}

} //namespace

