#ifndef ALGO_TILED_NOISE_H
#define ALGO_TILED_NOISE_H

#include <cmath>

namespace Algo {

//
// NOTE: If need 2D without time then tiledNoise2 is not-optimal since it
// calls getNoise3. If needed can provide another function
//


// 1D
double tiledNoise1(double x, int octaves, double mult, double w, double time=0);
// 2D
double tiledNoise2(double x, double y, int octaves, double mult, double w, double h, double time=0);
// 3D (no time based version)
double tiledNoise3(double x, double y, double z, int octaves, double mult, double w, double h, double d);

// Turb

// 1D
double tiledTurbulence1(double x, int octaves, double mult, double w, double time=0);
// 2D
double tiledTurbulence2(double x, double y, int octaves, double mult, double w, double h, double time=0);
// 3D (no time based version)
double tiledTurbulence3(double x, double y, double z, int octaves, double mult, double w, double h, double d);


// Simplex

// 1D
double tiledSNoise1(double x, int octaves, double mult, double w, double time=0);
// 2D
double tiledSNoise2(double x, double y, int octaves, double mult, double w, double h, double time=0);
// 3D 
double tiledSNoise3(double x, double y, double z, int octaves, double mult, double w, double h, double d, double time=0);
// 4D (no time based version)
double tiledSNoise4(double x, double y, double z, double ww, int octaves, double mult, double w, double h, double d, double t);

// Simplex Turb

// 1D
double tiledSTurbulence1(double x, int octaves, double mult, double w, double time=0);
// 2D
double tiledSTurbulence2(double x, double y, int octaves, double mult, double w, double h, double time=0);
// 3D
double tiledSTurbulence3(double x, double y, double z, int octaves, double mult, double w, double h, double d, double time=0);
// 4D (no time based version)
double tiledSTurbulence4(double x, double y, double z, double ww, int octaves, double mult, double w, double h, double d, double t);

} // namespace

#endif

