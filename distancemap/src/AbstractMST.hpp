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

	static void generateMSTAbstractEdges(const std::vector<GridType::Edge>& edges,
			const std::vector<GridType::Point>& baseNodes,
			const std::vector<GridType::AbstractNode>& abstractNodes,
			std::vector<GridType::AbstractEdge>& abstractEdges);
};

#endif /* DISTANCEMAP_SRC_ABSTRACTMST_H_ */
