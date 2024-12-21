#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <limits>
#include <cmath>

#include "BuildConfig.h"

namespace MathStuff {

#define MY_PI               3.14159265358979323846
#define DEG_TO_RAD(deg)     (((float)deg)* (MY_PI / 180.0f))
#define RAD_TO_DEG(rad)     (((float)rad)* (180.0f / MY_PI))

#define FLOATS_EQUAL(f1,f2) ( fabs(f2-f1) <= std::numeric_limits<float>::epsilon() * fabs(f1) )

inline long int float2int(float f)
{
#if defined(HAVE_LRINTF)
    return lrintf(f);
#elif defined(WIN32)
    int i;
    _asm
    {
        fld f
        fistp i
    };
    return i;
#else
    return ((int)(f+0.5f));
#endif
}
inline long int floor(float f) {
#if defined(WIN32)
    return float2int(f-0.5f);
#else
    return ::floorf(f);
#endif
}



// Double version (YUCK! just copied)
inline long int double2int(double f)
{
#if defined(HAVE_LRINTF)
    return lrint(f);
#elif defined(WIN32)
    int i;
    _asm
    {
        fld f
        fistp i
    };
    return i;
#else
    return ((int)(f));
#endif
}
inline long int floor(double f) {
#if defined(WIN32)
    return double2int(f-0.5);
#else
    return ::floor(f);
#endif
}


// Turn X,Y into Heading (0 = North, 90 = west)
inline int heading(float dx, float dy)
{
    int l_heading = float2int(RAD_TO_DEG(atanf(dy/dx)));
    l_heading += (dx  < 0) ? 90 : 270;
    return (l_heading == 360) ? 0 : l_heading;
}
// Turn heading (0 = North) into X Change (-1...1)
inline float xchange(float heading)
{
    return cosf(DEG_TO_RAD(heading+90));
}

// Turn heading (0 = North) into Y Change (-1...1)
inline float ychange(float heading)
{
    return sinf(DEG_TO_RAD(heading+90));
}

// Subtract two headings
inline int headingDiff(int lhs, int rhs)
{
    int l_diff = lhs - rhs;
    if (l_diff > 180)
    {
        l_diff = l_diff-360;
    }
    else
    if (l_diff < -180)
    {
       l_diff += 360;
    }
    // Need to negate because heading goes CCW
    return -l_diff;
}


} // namespace

#endif

