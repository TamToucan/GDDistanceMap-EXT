#include "PointsString.h"
#include "ConvString.h"
#include "MathStuff.h"

#include "Debug.h"

namespace MathStuff {

PointsString::PointsString(MathStuff::Coord defZ, bool defaultIsX) :
    m_defaultZorX(defZ),
    m_defaultIsX(defaultIsX)
{
}

PointsString::~PointsString()
{
}
void PointsString::setDefault(MathStuff::Coord def)
{
    m_defaultZorX = def;
}

MathStuff::Point PointsString::stringToPoint(const std::string& str) const
{
    MathStuff::Point l_pnt;
    std::vector<std::string> l_points;
    int l_numPoints = Util::splitString(str,",",l_points,false);
    if (l_numPoints != 2 and l_numPoints != 3)
    {
        LOG_ERROR("PointsString - Failed to parse " << str << " into X,Y,[Z] point");
    }
    else
    {
        l_pnt[1] = Util::decStrToFloat(l_points[1]);

        if (l_numPoints == 3)
        {
            l_pnt[0] = Util::decStrToFloat(l_points[0]);
            l_pnt[2] = Util::decStrToFloat(l_points[2]);
        }
        else
        {
            if (m_defaultIsX)
            {
                l_pnt[0] = m_defaultZorX;
                l_pnt[2] = Util::decStrToFloat(l_points[0]);
            }
            else
            {
                l_pnt[0] = Util::decStrToFloat(l_points[0]);
                l_pnt[2] = m_defaultZorX;
            }
        }
    }
    return l_pnt;
}

bool PointsString::parse(const std::string& theString, MathStuff::PointsList& thePoints, bool append) const
{
    // Split up on whitespace to get list of points
    std::vector<std::string> l_pointStrs;
    Util::tokenizeStr(theString, l_pointStrs);

    // Make space for the new points
    if (not append)
    {
        thePoints.clear();
    }
    thePoints.reserve(thePoints.size() + l_pointStrs.size());

    // Convert each point into X,Y,[Z] and store
    for (std::vector<std::string>::const_iterator l_itr = l_pointStrs.begin();
         l_itr != l_pointStrs.end();
         ++l_itr)
    {
        thePoints.push_back(stringToPoint(*l_itr));
    }
    return true;
}

} // namespace
