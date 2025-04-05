/*
 * AbstractMST.h
 *
 *  Created on: 22 Mar 2025
 *      Author: tam
 */

#ifndef DISTANCEMAP_SRC_ABSTRACTMST_H_
#define DISTANCEMAP_SRC_ABSTRACTMST_H_

#include "GridTypes.hpp"

class AbstractMST {
public:
	AbstractMST();
	virtual ~AbstractMST();

	static std::vector<GridType::AbstractEdge> generateMSTAbstractEdges(const GridType::BaseGraph& graph,
		const std::vector<GridType::Edge>& edges,
		const std::vector<GridType::Point>& baseNodes,
		const std::vector<GridType::AbstractNode>& abstractNodes);
};

#endif /* DISTANCEMAP_SRC_ABSTRACTMST_H_ */
