#ifndef UTIL_TRI_BOOL_H
#define UTIL_TRI_BOOL_H

namespace Util
{

class tribool
{
public:
    enum State { NO=0, YES=1, UNKNOWN=2 };
public:
    tribool() : m_state(UNKNOWN) { }
    explicit tribool(bool flag) : m_state(flag ? YES : NO) { }
    tribool(tribool::State s) : m_state(s) { }
    tribool(const tribool& tb) : m_state(tb.m_state) { }
    tribool::State state() const { return m_state; }

    const tribool& operator=(bool b) { m_state = (b ? YES : NO); return *this; }
private:
    State m_state;
};

inline
bool operator==(const tribool& rLHS, const tribool& rRHS) { return rLHS.state() == rRHS.state(); }

inline
bool operator==(const tribool& rLHS, const tribool::State& s) { return rLHS == tribool(s); }
inline
bool operator==(const tribool::State& s, const tribool& rRHS) { return tribool(s) == rRHS; }

inline
bool operator==(const tribool& rLHS, const bool& b) { return rLHS == tribool(b); }
inline
bool operator==(const bool& b, const tribool& rRHS) { return tribool(b) == rRHS; }
}
#endif
