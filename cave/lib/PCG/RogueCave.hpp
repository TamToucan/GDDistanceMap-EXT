#ifndef PCG_ROUGECAVE_H
#define PCG_ROUGECAVE_H

#include <vector>
#include <utility>
#include "Range.h"

namespace PCG {

class RogueCave {
public:
	RogueCave(int width, int height);
	virtual ~RogueCave() { }

	void addGeneration(Util::IntRange b3, Util::IntRange b5, Util::IntRange s3, Util::IntRange s5, int reps);

	std::vector<std::vector<int>>& generate();

    std::vector<std::vector<int>>& getGrid() { return grid; }

public:
    static constexpr int TILE_FLOOR = 0;
    static constexpr int TILE_WALL = 1;

    struct Generation {
    	Util::IntRange born3;
    	Util::IntRange survive3;
    	Util::IntRange born5;
    	Util::IntRange survive5;

    	int reps;
    	Generation() : reps(0) { }

    	Generation(Util::IntRange b3, Util::IntRange b5, Util::IntRange s3, Util::IntRange s5, int r) :
    		born3(b3),
    		born5(b5),
			survive3(s3),
			survive5(s5),
			reps(r)
    	{ }
    };

private:
    std::pair<int, int> count_neighbors(int x, int y);
	void generation(const RogueCave::Generation& params);

private:
    const int mWidth;
    const int mHeight;
    std::vector<Generation> mGenerations;

    std::vector<std::vector<int>> grid;
    std::vector<std::vector<int>> grid2;
};

}


#endif
