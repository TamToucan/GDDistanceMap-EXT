#ifndef MATHSTUFF_POINTS_STRING_H
#define MATHSTUFF_POINTS_STRING_H

#include <string>
#include "MathStuff.h"

namespace MathStuff {

//
// Parses a string of points in the form "X1,Y1,Z1  X2,Y2,Z2"
// If get two points then can either be
//  defaultIsX = false => X,Y and uses Z=m_defaultZorX
//  defaultIsX = true  => Z,Y and uses X=m_defaultZorX
//
class PointsString
{
public:
    explicit PointsString(MathStuff::Coord defZ=0.0, bool defaultIsX=false);
    virtual ~PointsString();

    virtual void setDefault(MathStuff::Coord def);

    // NOTE Always returns TRUE
    // If append==true then thePoints is not cleared
    //
    virtual bool parse(const std::string& theString, MathStuff::PointsList& thePoints, bool append=false) const;

    // Parse string into a single point
    virtual MathStuff::Point stringToPoint(const std::string& str) const;
protected:
    MathStuff::Coord m_defaultZorX;
    bool             m_defaultIsX;
};

// Helper classes so don't need to remember the horrible constructor above
//
class PointsStringXY : public PointsString
{
public:
    explicit PointsStringXY(MathStuff::Coord defZ=0.0) : PointsString(defZ, false) { }
};

class PointsStringZY : public PointsString
{
public:
    explicit PointsStringZY(MathStuff::Coord defX=0.0) : PointsString(defX, true) { }
};

} // namespace

#endif
