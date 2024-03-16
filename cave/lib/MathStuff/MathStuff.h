#ifndef MATHSTUFF_H
#define MATHSTUFF_H

#include <cmath>
#include <vector>

#include "Vector3.h"
#include "Vector2.h"

namespace MathStuff {

typedef float Coord;

typedef Vector3< MathStuff::Coord > Point;
typedef std::vector< MathStuff::Point > PointsList;

typedef Vector3< int > IntPoint;
typedef std::vector< MathStuff::IntPoint > IntPointsList;

typedef Vector2< MathStuff::Coord > Point2D;
typedef std::vector< MathStuff::Point2D > PointsList2D;

typedef Vector2< int > IntPoint2D;
typedef std::vector< MathStuff::IntPoint2D > IntPointsList2D;

// Currently used by Triangulate2D
struct TriIndex
{
    int i1;
    int i2;
    int i3;
    TriIndex(int iONE, int iTWO, int iTHREE) :
        i1(iONE), i2(iTWO), i3(iTHREE) { }
};
typedef std::vector< TriIndex > TriIndexList;

} // namespace

#endif

