#ifndef MATHSTUFF_SAFE_REALS_H
#define MATHSTUFF_SAFE_REALS_H

#include <iostream>
#include "MathUtils.h"
#include "MathStuff.h"

namespace MathStuff {

//
// Template for a SafeFloatDouble
//
template <typename TT>
struct SFD
{
    //
    // Constructors
    //
    TT m_value;
    SFD<TT>() : m_value(0.0f) { }
    SFD<TT>(const TT& v) : m_value(v) { }
    SFD<TT>(const SFD<TT>& rC) : m_value(rC.m_value) { }

    //
    // Methods to convert
    // NOTE: Don't overload cast because then get ambiguous operators
    //
    int asInt() const { return int((m_value > 0) ? (m_value+0.5) : (m_value-0.5)); }
    Coord asCoord() const { return m_value; }

    //
    // Operators
    // TODO: Fix this up, return const, move out of class etc.
    //
    const SFD<TT>& operator=(const SFD<TT>& rRHS) { m_value = rRHS.m_value; return *this; }

    bool operator==(const SFD<TT>& rRHS) const { return FLOATS_EQUAL(m_value,rRHS.m_value); }
    bool operator<(const SFD<TT>& rRHS) const { return (not (*this == rRHS)) and (m_value < rRHS.m_value); }

    bool operator!=(const SFD<TT>& rRHS) const { return not (*this == rRHS); }
    bool operator>(const SFD<TT>& rRHS) const { return (not (*this == rRHS)) and (not (*this < rRHS)); }
    bool operator>=(const SFD<TT>& rRHS) const { return (*this == rRHS) or (*this > rRHS); }
    bool operator<=(const SFD<TT>& rRHS) const { return (*this == rRHS) or (*this < rRHS); }

    SFD<TT> operator-() const { return SFD<TT>(-m_value); }

    SFD<TT> operator-(const SFD<TT>& rRHS) const { return SFD<TT>(m_value - rRHS.m_value); }
    SFD<TT> operator+(const SFD<TT>& rRHS) const { return SFD<TT>(m_value + rRHS.m_value); }
    SFD<TT> operator/(const SFD<TT>& rRHS) const { return SFD<TT>(m_value / rRHS.m_value); }
    SFD<TT> operator*(const SFD<TT>& rRHS) const { return SFD<TT>(m_value * rRHS.m_value); }

    SFD<TT>& operator-=(const SFD<TT>& rRHS) { m_value -= rRHS.m_value; return *this; }
    SFD<TT>& operator+=(const SFD<TT>& rRHS) { m_value += rRHS.m_value; return *this; }
    SFD<TT>& operator/=(const SFD<TT>& rRHS) { m_value /= rRHS.m_value; return *this; }
    SFD<TT>& operator*=(const SFD<TT>& rRHS) { m_value *= rRHS.m_value; return *this; }
};

//
// Operator ?= with RHS a "normal" type
//
template <typename TT, typename RHS_TT>
SFD<TT>& operator-=(SFD<TT>& rLHS, const RHS_TT& rRHS) { rLHS.m_value -= rRHS; return rLHS; }
template <typename TT, typename RHS_TT>
SFD<TT>& operator+=(SFD<TT>& rLHS, const RHS_TT& rRHS) { rLHS.m_value += rRHS; return rLHS; }
template <typename TT, typename RHS_TT>
SFD<TT>& operator/=(SFD<TT>& rLHS, const RHS_TT& rRHS) { rLHS.m_value /= rRHS; return rLHS; }
template <typename TT, typename RHS_TT>
SFD<TT>& operator*=(SFD<TT>& rLHS, const RHS_TT& rRHS) { rLHS.m_value *= rRHS; return rLHS; }

//
// Operators -+/* with RHS a "normal" type
//
template<typename TT, typename RHS_TT>
    SFD<TT> operator-(const SFD<TT>& rLHS, const RHS_TT& rRHS) { return SFD<TT>(rLHS.m_value - rRHS); }
template<typename TT, typename RHS_TT>
    SFD<TT> operator+(const SFD<TT>& rLHS, const RHS_TT& rRHS) { return SFD<TT>(rLHS.m_value + rRHS); }
template<typename TT, typename RHS_TT>
    SFD<TT> operator/(const SFD<TT>& rLHS, const RHS_TT& rRHS) { return SFD<TT>(rLHS.m_value / rRHS); }
template<typename TT, typename RHS_TT>
    SFD<TT> operator*(const SFD<TT>& rLHS, const RHS_TT& rRHS) { return SFD<TT>(rLHS.m_value * rRHS); }

//
// Operators -+/* with LHS a "normal" type
//
template<typename LHS_TT, typename TT>
    SFD<TT> operator-(const LHS_TT& rLHS, const SFD<TT>& rRHS) { return SFD<TT>(rLHS - rRHS.m_value); }
template<typename LHS_TT, typename TT>
    SFD<TT> operator+(const LHS_TT& rLHS, const SFD<TT>& rRHS) { return SFD<TT>(rLHS + rRHS.m_value); }
template<typename LHS_TT, typename TT>
    SFD<TT> operator/(const LHS_TT& rLHS, const SFD<TT>& rRHS) { return SFD<TT>(rLHS / rRHS.m_value); }
template<typename LHS_TT, typename TT>
    SFD<TT> operator*(const LHS_TT& rLHS, const SFD<TT>& rRHS) { return SFD<TT>(rLHS * rRHS.m_value); }

//
// I/O stream
//
template<typename TT>
std::istream& operator>>(std::istream& rIS, const SFD<TT>& rSFD) { return rIS >> rSFD.m_value; }
template<typename TT>
std::ostream& operator<<(std::ostream& rOS, const SFD<TT>& rSFD) { return rOS << rSFD.m_value; }

//
// Define some types
//
typedef SFD<float> Float;
typedef SFD<double> Double;

} // namespace

#endif
