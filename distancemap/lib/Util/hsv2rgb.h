#ifndef UTIL_HSV_TO_RGB_H
#define UTIL_HSV_TO_RGB_H

#include <math.h>

namespace Util {

#define RETURN_RGB(red, green, blue) {r=int(255*red+0.5); g=int(255*green+0.5); b=int(255*blue+0.5); return; }

//
// H,S,V in range 0..1
// Returns R,G,B in range 0..255
inline
void hsv2rgb( float h, float s, float v, int& r, int& g, int& b) {
        float m,n,f;
    h *= 6.;
    int i = int(floor(h));
    f = h - i;
    if ( !(i&1) ) f = 1. - f;
    m = v * (1. - s);
    n = v * (1. - s * f);
    switch (i) {
        case 6:
        case 0: RETURN_RGB(v, n, m);
        case 1: RETURN_RGB(n, v, m);
        case 2: RETURN_RGB(m, v, n)
        case 3: RETURN_RGB(m, n, v);
        case 4: RETURN_RGB(n, m, v);
        case 5: RETURN_RGB(v, m, n);
    }
}

}
#endif
