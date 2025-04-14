#include <fstream>
#include <iostream>
#include <string>
#include "GridToGraph.hpp"


int main(int argc, char** argv)
{
	auto grid = GridToGraph::readGridFromFile("D:/tmp/GRID.txt");
	auto graph = GridToGraph::makeGraph(grid);
	return 0;
}