#include <cmath>

#include "PerlinNoise.h"
#include "MathUtils.h"
#include "noise1234.h"
namespace Algo {

double noise2(double x, double y) {
	return ::snoise2(static_cast<float>(x), static_cast<float>(y));
}
double noise3(double x, double y, double z) {
	return ::noise3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

#ifdef BLAH

namespace {

static int p[512];
static int permutation[256] =
{
    151,160,137,91,90,15,
    131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
    190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
    88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
    77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
    102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
    135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
    5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
    223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
    129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
    251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
    49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
    138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
};

class Initor
{
public:
    Initor()
    {
        for (int i=0; i < 256 ; i++)
        {
            p[256+i] = p[i] = permutation[i];
        }
    }
};

static Initor init_permutations;

} // anonymouse


inline
static double fade(double t)
{
    return t * t * t * (t * (t * 6 - 15) + 10);
}

inline
static double lerp(double t, double a, double b)
{
    return a + t * (b - a);
}

inline
static double grad(int h, double x, double y)
{
    switch (h&0xf)
    {
    case 0x0: return x+y;
    case 0x1: return -x+y;
    case 0x2: return x-y;
    case 0x3: return -x-y;
    case 0x4: return x;
    case 0x5: return -x;
    case 0x6: return x;
    case 0x7: return -x;
    case 0x8: return y;
    case 0x9: return -y;
    case 0xa: return y;
    case 0xb: return -y;
    case 0xc: return x+y;
    case 0xd: return -y;
    case 0xe: return -x+y;
    case 0xf: return -y;
    }
    return 0;
}

inline
static double grad(int h, double x, double y, double z)
{
    switch (h&0xf)
    {
    case 0x0: return x+y;
    case 0x1: return -x+y;
    case 0x2: return x-y;
    case 0x3: return -x-y;
    case 0x4: return x+z;
    case 0x5: return -x+z;
    case 0x6: return x-z;
    case 0x7: return -x-z;
    case 0x8: return y+z;
    case 0x9: return -y+z;
    case 0xa: return y-z;
    case 0xb: return -y-z;
    case 0xc: return x+y;
    case 0xd: return -y+z;
    case 0xe: return -x+y;
    case 0xf: return -y-z;
    }
    return 0;
}

double noise2(double x, double y)
{
    // FIND UNIT CUBE THAT CONTAINS POINT.
    int X = MathStuff::floor(x) & 255;
    int Y = MathStuff::floor(y) & 255;
    // FIND RELATIVE X,Y,Z OF POINT IN CUBE.
    x -= MathStuff::floor(x);
    y -= MathStuff::floor(y);
    // COMPUTE FADE CURVES FOR EACH OF X,Y
    double u = fade(x);
    double v = fade(y);
    // HASH COORDINATES OF THE 4 SQUARE CORNERS,
    int A  = p[X  ]+Y;
    int AA = p[A  ];
    int AB = p[A+1];

    int B  = p[X+1]+Y;
    int BA = p[B];
    int BB = p[B+1];

    // AND ADD BLENDED RESULTS FROM 4 CORNERS OF SQUARE
    return 
            lerp(v,
                lerp(u,
                    grad(p[AA  ], x  , y   ),
                    grad(p[BA  ], x-1, y   )
                    ),
                lerp(u,
                    grad(p[AB  ], x  , y-1),
                    grad(p[BB  ], x-1, y-1)
                    )
                );
}

double noise3(double x, double y, double z)
{
    // FIND UNIT CUBE THAT CONTAINS POINT.
    int X = MathStuff::floor(x) & 255;
    int Y = MathStuff::floor(y) & 255;
    int Z = MathStuff::floor(z) & 255;
    // FIND RELATIVE X,Y,Z OF POINT IN CUBE.
    x -= MathStuff::floor(x);
    y -= MathStuff::floor(y);
    z -= MathStuff::floor(z);
    // COMPUTE FADE CURVES FOR EACH OF X,Y,Z.
    double u = fade(x);
    double v = fade(y);
    double w = fade(z);
    // HASH COORDINATES OF THE 8 CUBE CORNERS,
    int A  = p[X  ]+Y;
    int AA = p[A  ]+Z;
    int AB = p[A+1]+Z;

    int B  = p[X+1]+Y;
    int BA = p[B]+Z;
    int BB = p[B+1]+Z;

    // AND ADD BLENDED RESULTS FROM 8 CORNERS OF CUBE
    return lerp(w,
            lerp(v,
                lerp(u,
                    grad(p[AA  ], x  , y  , z   ),
                    grad(p[BA  ], x-1, y  , z   )
                    ),
                lerp(u,
                    grad(p[AB  ], x  , y-1, z   ),
                    grad(p[BB  ], x-1, y-1, z   )
                    )
                ),
            lerp(v,
                lerp(u,
                    grad(p[AA+1], x  , y  , z-1 ),
                    grad(p[BA+1], x-1, y  , z-1 )
                    ),
                lerp(u,
                    grad(p[AB+1], x  , y-1, z-1 ),
                    grad(p[BB+1], x-1, y-1, z-1 ))
                )
            );
}


#endif

} // namespace

