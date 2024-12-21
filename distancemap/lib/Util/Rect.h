#ifndef UTIL_RECT_H
#define UTIL_RECT_H

#include <iostream>
#include "Vector2.h"

//
// If width/height == 0 then it won't contain or overlap X,Y
//
namespace Geom {

template <typename T>
class Rect
{
public:
    Rect() : m_x(0), m_y(0), m_width(0), m_height(0) { }
    Rect(const Rect<T>& r) { clone(r); }
    Rect(const T& x, const T& y, const T& w, const T& h)
        : m_x(x), m_y(y), m_width(w), m_height(h) { }
    ~Rect() { }

    const T& getX() const { return m_x; }
    const T& getY() const { return m_y; }
    const T& getWidth() const { return m_width; }
    const T& getHeight() const { return m_height; }
    const T getRightX() const { return m_x+m_width; }
    const T getTopY() const { return m_y+m_height; }
    const T getMiddleX() const { return m_x+m_width/2; }
    const T getMiddleY() const { return m_y+m_height/2; }

    void setX(const T& x) { m_x = x; }
    void setY(const T& y) { m_y = y; }
    void setWidth(const T& w) { m_width = w; }
    void setHeight(const T& h) { m_height = h; }

    const Rect<T>& operator=(const Rect<T>& rRHS) { clone(rRHS); return *this; }
    void clone(const Rect<T>& r)
    {
        m_x = r.m_x;
        m_y = r.m_y;
        m_width = r.m_width;
        m_height = r.m_height;
    }
    bool contains(const T& x, const T& y) const
    {
        return (x >= m_x) && (x < (m_x+m_width)) && (y >= m_y) && (y < (m_y+m_height));
    }
    bool overlaps(const Rect<T>& r) const
    {
        return (   contains(r.getX(), r.getY())
                or contains(r.getRightX(), r.getY())
                or contains(r.getRightX(), r.getTopY())
                or contains(r.getX(), r.getTopY()) );
    }
    void growToContain(const T& x, const T& y) {
        if (x < m_x) {
        	m_x = x;
        }
        else if (x >= (m_x+m_width)) {
        	m_width = 1+(x - m_x);
        }
        if (y < m_y) {
        	m_y = y;
        }
        else if (y >= (m_y+m_height)) {
        	m_height = 1+(y - m_y);
        }
    }
    void centerAt(const MathStuff::Vector2< T >& pnt)
    {
        m_x = pnt[0]-m_width/2;
        m_y = pnt[1]-m_height/2;
    }
    void grow(const T& c) { grow(c, c); }
    void grow(const T& w, const T& h)
    {
        m_x -= w;
        m_y -= h;
        m_width += w; m_width += w;
        m_height += h; m_height += h;
    }
    void shrink(const T& c) { shrink(c, c); }
    void shrink(const T& w, const T& h) { grow(-w, -h); }
protected:
    T m_x;
    T m_y;
    T m_width;
    T m_height;
};

typedef Rect<int> IntRect;
typedef Rect<float> FloatRect;

// Helper cout (for debug)
template<typename T>
std::ostream& operator<<(std::ostream& os, const Geom::Rect<T>& r) {
    return os << r.getX() << ',' << r.getY() << ':' << r.getWidth() << ',' << r.getHeight();
}

} // namespace

#endif
