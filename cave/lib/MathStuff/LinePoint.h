#ifndef MATHSTUFF_LINE_POINT_H
#define MATHSTUFF_LINE_POINT_H

#include "CLAMP.h"
#include "MathStuff.h"
#include "MathUtils.h"

namespace MathStuff {

//
// Access points on a line
//
class LinePoint
{
public:
    LinePoint(const MathStuff::Point& s, const MathStuff::Point& e)
        : m_start(s)
        , m_end(e)
        , m_diff(e-s)
    {
    }

    ~LinePoint() { }

    // Ratio = 0 for start, 1 for end, 2 for twice the distance
    // of start to end, -ve for opposite direction, etc
    MathStuff::Point get(float ratio) const
    {
        return m_start + m_diff*ratio;
    }
private:
    const MathStuff::Point m_start;
    const MathStuff::Point m_end;
    const MathStuff::Point m_diff;
};

} // namespace

#endif
