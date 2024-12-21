#ifndef UTIL_CLAMP_H
#define UTIL_CLAMP_H

#include <cmath>

namespace Util {

template<typename T> T MIN(const T& rV, const T& rMin)
{
    return (rV < rMin)
           ? rV
           : rMin;
}

template<typename T> T MAX(const T& rV, const T& rMax)
{
    return (rV > rMax)
           ? rV
           : rMax;
}

template<typename T> T CLAMP(const T& rV, const T& rMin, const T& rMax)
{
    return (rV < rMin)
           ? rMin
           : (rV > rMax)
             ? rMax
             : rV;
}

// rV has been decreased, wrap it to rMax if < rMin
template<typename T> T WRAP_DEC(const T& rV, const T& rMin, const T& rMax)
{
    return (rV < rMin)
           ? rMax
           : rV;
}

// rV has been increased, wrap it to rMin if > rMax
template<typename T> T WRAP_INC(const T& rV, const T& rMin, const T& rMax)
{
    return (rV > rMax)
           ? rMin
           : rV;
}


// If rV < rMin warp to rMax, if rV > rMax wrap to rMin
template<typename T> T WRAP(const T& rV, const T& rMin, const T& rMax)
{
    return (rV < rMin)
           ? rMax
           : (rV > rMax)
             ? rMin
             : rV;
}

// NOTE: LOOP_INC/DEC are for min..max-1
// rV has been increased, reduce it until < rMax
template<typename T> T LOOP_INC(const T& rV, const T& rMin, const T& rMax)
{
    T l_ret(rV);
    const T l_adj(rMax-rMin);
    while (l_ret >= rMax) l_ret -= l_adj;
    return l_ret;
}


// rV has been decreased, increase it until >= min
template<typename T> T LOOP_DEC(const T& rV, const T& rMin, const T& rMax)
{
    T l_ret(rV);
    const T l_adj(rMax-rMin);
    while (l_ret <= rMin) l_ret += l_adj;
    if (l_ret >= rMax) l_ret -= l_adj;
    return l_ret;
}

// rV has changed, if outside rMin or rMax then loop round until in range
template<typename T> T LOOP(const T& rV, const T& rMin, const T& rMax)
{
    return (rV >= rMax)
        ? LOOP_INC(rV, rMin, rMax)
        : LOOP_DEC(rV, rMin, rMax);
}



template<typename T> T ABS(const T& rV)
{
    return (rV >= T(0))
           ? rV
           : -rV;
}
template<> inline float ABS(const float& rV) { return fabsf(rV); }
template<> inline double ABS(const double& rV) { return fabs(rV); }
template<> inline long double ABS(const long double& rV) { return fabsl(rV); }

//
// Return percetage of value (hides the casts)
//
template<typename T> T PERCENT_OF(const T& p, const T& rV)
{
    return T((double(p) * double(rV)) / 100.);
}

//
// Convert value to percetage
//
template<typename T> T AS_PERCENT(const T& rV1, const T& rV2)
{
    return T((double(rV1) / double(rV2)) * 100.);
}

//
// compute the next highest power of 2 of 32-bit v
// If given a power of two, returns that value
// i.e. add one to v if you really want the next
inline
unsigned int NEXT_POW2(unsigned int v)
{
    --v;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return ++v;
}

// 
// True if value >= Min and <= Max
//
template<typename T> bool BETWEEN(const T& rV, const T& rMin, const T& rMax)
{
    return (rV >= rMin) and (rV <= rMax);
}

//
// Number of digits for number
//
template<typename T> int NUM_DIGITS(const T& rV)
{
    return int(std::ceil(std::log10(Util::ABS(rV)+1)));
}

//
// Inverse clamp. If rV between min...max then return which it is closer too
//
template<typename T> T INV_CLAMP(const T& rV, const T& rMin, const T& rMax)
{
    return BETWEEN(rV, rMin, rMax)
           ? (rV-rMin) < (rMax-rV)
             ? rMin
             : rMax
           : rV;
}


} // namespace
#endif

