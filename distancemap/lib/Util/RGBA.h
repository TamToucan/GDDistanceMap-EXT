#ifndef UTIL_RGBA_H
#define UTIL_RGBA_H

#include "CLAMP.h"

namespace Util {

    // Class to represent a color. Really just to make things more readable

// NOTE: No virtual destructor, not virtual functions
class RGBA
{
public:
    RGBA()
    {
        rgba[0] = 0x00;
        rgba[1] = 0x00;
        rgba[2] = 0x00;
        rgba[3] = 0x00;
    }
    RGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
    {
        rgba[0] = r;
        rgba[1] = g;
        rgba[2] = b;
        rgba[3] = a;
    }
    RGBA(float r, float g, float b, float a)
    {
        rgba[0] = static_cast<unsigned char>(255.0f * Util::MIN(r,1.0f));
        rgba[1] = static_cast<unsigned char>(255.0f * Util::MIN(g,1.0f));
        rgba[2] = static_cast<unsigned char>(255.0f * Util::MIN(b,1.0f));
        rgba[3] = static_cast<unsigned char>(255.0f * Util::MIN(a,1.0f));
    }
    // NOTE: Not explicit
    RGBA(unsigned long color)
    {
        clone(color);
    }
    const RGBA& operator=(unsigned long color)
    {
        clone(color);
        return *this;
    }
    const RGBA& operator=(const RGBA& rRHS)
    {
        clone(rRHS.asULong());
        return *this;
    }

    const unsigned char r() const { return rgba[0]; }
    const unsigned char g() const { return rgba[1]; }
    const unsigned char b() const { return rgba[2]; }
    const unsigned char a() const { return rgba[3]; }

    const float r_f() const { return float(rgba[0])/255.0f; }
    const float g_f() const { return float(rgba[1])/255.0f; }
    const float b_f() const { return float(rgba[2])/255.0f; }
    const float a_f() const { return float(rgba[3])/255.0f; }

    const unsigned long asULong() const
    {
        return static_cast<unsigned long>(rgba[0])<<24
             | static_cast<unsigned long>(rgba[1])<<16
             | static_cast<unsigned long>(rgba[2])<<8
             | static_cast<unsigned long>(rgba[3])<<0;
    }
    void lerpTo(const RGBA& to, float ratio)
    {
        const unsigned char l_0xFF(0xff);
        rgba[0] = (to.r() > r())
        ? Util::MIN(static_cast<unsigned char>(r() + ((to.r()-r())*ratio)), l_0xFF)
        : Util::MIN(static_cast<unsigned char>(r() - ((r()-to.r())*ratio)), l_0xFF);
        rgba[1] = (to.g() > g())
        ? Util::MIN(static_cast<unsigned char>(g() + ((to.g()-g())*ratio)), l_0xFF)
        : Util::MIN(static_cast<unsigned char>(g() - ((g()-to.g())*ratio)), l_0xFF);
        rgba[2] = (to.b() > b())
        ? Util::MIN(static_cast<unsigned char>(b() + ((to.b()-b())*ratio)), l_0xFF)
        : Util::MIN(static_cast<unsigned char>(b() - ((b()-to.b())*ratio)), l_0xFF);
        rgba[3] = (to.a() > a())
        ? Util::MIN(static_cast<unsigned char>(a() + ((to.a()-a())*ratio)), l_0xFF)
        : Util::MIN(static_cast<unsigned char>(a() - ((a()-to.a())*ratio)), l_0xFF);
    }


private:
    unsigned char rgba[4];

    void clone(unsigned long color)
    {
        rgba[0] = (color&0xff000000)>>24;
        rgba[1] = (color&0x00ff0000)>>16;
        rgba[2] = (color&0x0000ff00)>>8;
        rgba[3] = (color&0x000000ff)>>0;
    }
};

} // namespace

#endif
