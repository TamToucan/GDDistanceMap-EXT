#ifndef MATHSTUFF_CATMULL_ROM_SPLINE_H
#define MATHSTUFF_CATMULL_ROM_SPLINE_H

#include "MathStuff.h"

namespace MathStuff {

class CatmullRomSpline
{
public:
    // Skip Z is to only calc X,Y in getPoint
    CatmullRomSpline(const MathStuff::PointsList& path, bool skipZ=false);

    // Given a time T (0..1) return the point on the path
    MathStuff::Point getPoint(float t) const;

private:
    MathStuff::Point interpolatePoints(int i, int j, int k, int l, float t) const;

private:
    const MathStuff::PointsList m_path;
    const bool m_skipZ;
};

} // namespace

#endif

