#ifndef MATHSTUFF_LINE_MOVER_H
#define MATHSTUFF_LINE_MOVER_H

#include "CLAMP.h"
#include "MathUtils.h"

namespace MathStuff {

//
// Moves from ox,oy in a straight line towards nx,ny
//
class LineMover
{
public:
    LineMover()
    {
        init(0,0,0,0);
    }
    LineMover(int ox, int oy, int nx, int ny)
    {
        init(ox,oy, nx,ny);
    }

    LineMover(int dir, int x, int y)
    {
        init(dir, x,y);
    }

    void init (int dir, int x, int y)
    {
        // Pick a point 100k away from x,y
        int nx = x + int(100000 * MathStuff::xchange(dir));
        int ny = y + int(100000 * MathStuff::ychange(dir));
        init(x,y, nx,ny);
    }

    void init (int ox, int oy, int nx, int ny)
    {
        m_ox = ox;
        m_oy = oy;
        m_nx = nx;
        m_ny = ny;

        m_dx = Util::ABS(m_nx-m_ox);
        m_dy = Util::ABS(m_ny-m_oy);
        m_sx = (m_ox < m_nx) ? 1 : -1;
        m_sy = (m_oy < m_ny) ? 1 : -1;
        m_curx = m_ox;
        m_cury = m_oy;
        m_err = m_dx-m_dy;
    }
    ~LineMover() { }

    // Return TRUE if still more step to reach destination
    bool move(int steps=1)
    {
        while (steps-- and ((m_curx != m_nx) or (m_cury != m_ny)))
        {
            int e2 = 2*m_err;
            if (e2 > -m_dy)
            {
                m_err -= m_dy;
                m_curx += m_sx;
            }
            if (e2 < m_dx)
            {
                m_err += m_dx;
                m_cury += m_sy;
            }
        }
        return ((m_curx != m_nx) or (m_cury != m_ny));
    }
    int getX() const { return m_curx; }
    int getY() const { return m_cury; }
    int getStartX() const { return m_ox; }
    int getStartY() const { return m_oy; }
    int getEndX() const { return m_nx; }
    int getEndY() const { return m_ny; }
    int getDeltaX() const { return m_nx-m_curx; }
    int getDeltaY() const { return m_ny-m_cury; }
private:
   int m_ox;
   int m_oy;
   int m_nx;
   int m_ny;
   int m_dx;
   int m_dy;
   int m_sx;
   int m_sy;
   int m_curx;
   int m_cury;
   int m_err;
};

} // namespace

#endif
