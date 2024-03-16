#include "CatmullRomSpline.h"
#include "MathStuff.h"
#include "CLAMP.h"

#include "Debug.h"


namespace MathStuff {

namespace {

inline
float catmullRomSpline(float a, float b, float c, float d, float t)
{
    return 0.5 *(
                   (2 * b) +
                   (-a + c) * t +
                   (2*a - 5*b + 4*c - d) * t * t +
                   (-a + 3*b- 3*c + d) * t * t * t);
}

} // anonymouse

CatmullRomSpline::CatmullRomSpline(const MathStuff::PointsList& path, bool skipZ)
    : m_path(path)
    , m_skipZ(skipZ)
{
    LOG_ASSERT(!m_path.empty(),"EEK! Spline path empty");
}

MathStuff::Point CatmullRomSpline::getPoint(float t) const
{
    int l_segment = Util::CLAMP(int((m_path.size()-1) * t), 0, int(m_path.size())-2);

    t = (t * (m_path.size()-1)) - l_segment;

    // If 1st l_segment then use 1st point as p-1
    // If end l_segment then use last point as p+2
    // else use p-1,p,p+1,p+2
    return (l_segment == 0)
        ? interpolatePoints(0, 0, 1, 2, t)
        : (l_segment == int(m_path.size())-2)
           ? interpolatePoints(m_path.size()-3, m_path.size()-2, m_path.size()-1, m_path.size()-1, t)
           : interpolatePoints(l_segment-1, l_segment, l_segment+1, l_segment+2, t);
}

MathStuff::Point CatmullRomSpline::interpolatePoints(int i, int j, int k, int l, float t) const
{
    MathStuff::Point l_retVal;
    for (int xyz=0; xyz < (m_skipZ ? 2 : 3); ++xyz)
    {
        l_retVal[xyz] = catmullRomSpline(m_path[i][xyz], m_path[j][xyz], m_path[k][xyz], m_path[l][xyz], t);
    }
    return l_retVal;
}

} // namespace

