#include "TiledNoise.h"
#include "PerlinNoise.h"
#include "SimplexNoise.h"

namespace Algo {

//
// 1D
//
double tiledNoise1(double x, int octaves, double mult, double w, double time)
{
    double l_noise =  (
            (    getNoise2(x/w*mult,time,octaves)
                    * (w - x) )
            + (    getNoise2((x-w)/w*mult,time,octaves)
                    * (x) )
    ) / (w);
    return l_noise;
}

//
// 2D
//
double tiledNoise2NoTime(double x, double y, int octaves, double mult, double w, double h)
{
    double l_noise =  (
            (    getNoise2(x/w*mult,y/h*mult,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getNoise2((x-w)/w*mult,y/h*mult,octaves)
                    * (x)
                    * (h - y) )
            + (    getNoise2((x-w)/w*mult,(y-h)/h*mult,octaves)
                    * (x)
                    * (y) )
            + (    getNoise2(x/w*mult,(y-h)/h*mult,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

double tiledNoise2(double x, double y, int octaves, double mult, double w, double h, double time)
{
    if (time == 0) return tiledNoise2NoTime(x,y,octaves,mult,w,h);

    double l_noise =  (
            (    getNoise3(x/w*mult,y/h*mult,time,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getNoise3((x-w)/w*mult,y/h*mult,time,octaves)
                    * (x)
                    * (h - y) )
            + (    getNoise3((x-w)/w*mult,(y-h)/h*mult,time,octaves)
                    * (x)
                    * (y) )
            + (    getNoise3(x/w*mult,(y-h)/h*mult,time,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

//
// 3D
//
double tiledNoise3(double x, double y, double z, int octaves, double mult, double w, double h, double d)
{
    double l_noise = (
            (    getNoise3(x/w*mult,y/h*mult,z/d*mult,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z) )
            + (    getNoise3((x-w)/w*mult,y/h*mult,z/d*mult,octaves)
                * (x)
                * (h - y)
                * (d - z) )
            + (    getNoise3((x-w)/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (x)
                * (y)
                * (d - z) )
            + (    getNoise3(x/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (w - x)
                * (y)
                * (d - z) )


            + (    getNoise3(x/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                * (w - x)
                * (h - y)
                * (z) )
            + (    getNoise3((x-w)/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (h - y)
                    * (z) )
            + (    getNoise3((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (y)
                    * (z) )
            + (    getNoise3(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (w - x)
                    * (y)
                    * (z) )

            ) / (w*h*d) ;
    return l_noise;
}


//
// 1D
//
double tiledTurbulence1(double x, int octaves, double mult, double w, double time)
{
    double l_noise =  (
            (    getTurbulence2(x/w*mult,time,octaves)
                    * (w - x) )
            + (    getTurbulence2((x-w)/w*mult,time,octaves)
                    * (x) )
    ) / (w);
    return l_noise;
}

//
// 2D
//
double tiledTurbulence2NoTime(double x, double y, int octaves, double mult, double w, double h)
{
    double l_noise =  (
            (    getTurbulence2(x/w*mult,y/h*mult,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getTurbulence2((x-w)/w*mult,y/h*mult,octaves)
                    * (x)
                    * (h - y) )
            + (    getTurbulence2((x-w)/w*mult,(y-h)/h*mult,octaves)
                    * (x)
                    * (y) )
            + (    getTurbulence2(x/w*mult,(y-h)/h*mult,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

double tiledTurbulence2(double x, double y, int octaves, double mult, double w, double h, double time)
{
    if (time == 0) return tiledTurbulence2NoTime(x,y,octaves,mult,w,h);

    double l_noise =  (
            (    getTurbulence3(x/w*mult,y/h*mult,time,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getTurbulence3((x-w)/w*mult,y/h*mult,time,octaves)
                    * (x)
                    * (h - y) )
            + (    getTurbulence3((x-w)/w*mult,(y-h)/h*mult,time,octaves)
                    * (x)
                    * (y) )
            + (    getTurbulence3(x/w*mult,(y-h)/h*mult,time,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

//
// 3D
//
double tiledTurbulence3(double x, double y, double z, int octaves, double mult, double w, double h, double d)
{
    double l_noise = (
            (    getTurbulence3(x/w*mult,y/h*mult,z/d*mult,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z) )
            + (    getTurbulence3((x-w)/w*mult,y/h*mult,z/d*mult,octaves)
                * (x)
                * (h - y)
                * (d - z) )
            + (    getTurbulence3((x-w)/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (x)
                * (y)
                * (d - z) )
            + (    getTurbulence3(x/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (w - x)
                * (y)
                * (d - z) )


            + (    getTurbulence3(x/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                * (w - x)
                * (h - y)
                * (z) )
            + (    getTurbulence3((x-w)/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (h - y)
                    * (z) )
            + (    getTurbulence3((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (y)
                    * (z) )
            + (    getTurbulence3(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (w - x)
                    * (y)
                    * (z) )

            ) / (w*h*d) ;
    return l_noise;
}

//===============================================================
// SIMPLEX VERSIONS
//===============================================================

//
// 1D
//
double tiledSNoise1(double x, int octaves, double mult, double w, double time)
{
    double l_noise =  (
            (    getSNoise2(x/w*mult,time,octaves)
                    * (w - x) )
            + (    getSNoise2((x-w)/w*mult,time,octaves)
                    * (x) )
    ) / (w);
    return l_noise;
}

//
// 2D
//
double tiledSNoise2NoTime(double x, double y, int octaves, double mult, double w, double h)
{
    double l_noise =  (
            (    getSNoise2(x/w*mult,y/h*mult,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getSNoise2((x-w)/w*mult,y/h*mult,octaves)
                    * (x)
                    * (h - y) )
            + (    getSNoise2((x-w)/w*mult,(y-h)/h*mult,octaves)
                    * (x)
                    * (y) )
            + (    getSNoise2(x/w*mult,(y-h)/h*mult,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

double tiledSNoise2(double x, double y, int octaves, double mult, double w, double h, double time)
{
    if (time==0) return tiledSNoise2NoTime(x,y,octaves,mult,w,h);
    double l_noise =  (
            (    getSNoise3(x/w*mult,y/h*mult,time,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getSNoise3((x-w)/w*mult,y/h*mult,time,octaves)
                    * (x)
                    * (h - y) )
            + (    getSNoise3((x-w)/w*mult,(y-h)/h*mult,time,octaves)
                    * (x)
                    * (y) )
            + (    getSNoise3(x/w*mult,(y-h)/h*mult,time,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

//
// 3D
//
double tiledSNoise3NoTime(double x, double y, double z, int octaves, double mult, double w, double h, double d)
{
    double l_noise = (
            (    getSNoise3(x/w*mult,y/h*mult,z/d*mult,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z) )
            + (    getSNoise3((x-w)/w*mult,y/h*mult,z/d*mult,octaves)
                * (x)
                * (h - y)
                * (d - z) )
            + (    getSNoise3((x-w)/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (x)
                * (y)
                * (d - z) )
            + (    getSNoise3(x/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (w - x)
                * (y)
                * (d - z) )


            + (    getSNoise3(x/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                * (w - x)
                * (h - y)
                * (z) )
            + (    getSNoise3((x-w)/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (h - y)
                    * (z) )
            + (    getSNoise3((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (y)
                    * (z) )
            + (    getSNoise3(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (w - x)
                    * (y)
                    * (z) )

            ) / (w*h*d) ;
    return l_noise;
}

double tiledSNoise3(double x, double y, double z, int octaves, double mult, double w, double h, double d, double time)
{
    if (time == 0) return tiledSNoise3NoTime(x,y,z,octaves,mult,w,h,d);
    double l_noise = (
            (    getSNoise4(x/w*mult,y/h*mult,z/d*mult,time,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z) )
            + (    getSNoise4((x-w)/w*mult,y/h*mult,z/d*mult,time,octaves)
                * (x)
                * (h - y)
                * (d - z) )
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,z/d*mult,time,octaves)
                * (x)
                * (y)
                * (d - z) )
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,z/d*mult,time,octaves)
                * (w - x)
                * (y)
                * (d - z) )


            + (    getSNoise4(x/w*mult,y/h*mult,(z-d)/d*mult,time,octaves)
                * (w - x)
                * (h - y)
                * (z) )
            + (    getSNoise4((x-w)/w*mult,y/h*mult,(z-d)/d*mult,time,octaves)
                    * (x)
                    * (h - y)
                    * (z) )
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,time,octaves)
                    * (x)
                    * (y)
                    * (z) )
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,time,octaves)
                    * (w - x)
                    * (y)
                    * (z) )

            ) / (w*h*d) ;
    return l_noise;
}

//
// 4D
//
double tiledNoise4(double x, double y, double z, double ww, int octaves, double mult, double w, double h, double d, double t)
{
    double l_noise = (
            (    getSNoise4(x/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z)
                 * (t - ww))
            + (    getSNoise4((x-w)/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (d - z)
                * (t - ww))
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (d - z)
                * (t - ww))
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (d - z)
                * (t - ww))


            + (    getSNoise4(x/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (h - y)
                * (z)
                * (t - ww))
            + (    getSNoise4((x-w)/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (z)
                * (t - ww))
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (z)
                * (t - ww))
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (z)
                * (t - ww))
                    
            + (  getSNoise4(x/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (h - y)
                * (d - z)
                * (ww))
            + (    getSNoise4((x-w)/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (d - z)
                * (ww))
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (d - z)
                * (ww))
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (d - z)
                * (ww))


            + (    getSNoise4(x/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (h - y)
                * (z)
                * (ww))
            + (    getSNoise4((x-w)/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (z)
                * (ww))
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (z)
                * (ww))
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (z)
                * (ww))

            ) / (w*h*d*t) ;
    return l_noise;
}


//===================
// Turb
//==================

//
// 1D
//
double tiledSTurbulence1(double x, int octaves, double mult, double w, double time)
{
    double l_noise =  (
            (    getSTurbulence2(x/w*mult,time,octaves)
                    * (w - x) )
            + (  getSTurbulence2((x-w)/w*mult,time,octaves)
                    * (x) )
    ) / (w);
    return l_noise;
}


//
// 2D
//
double tiledSTurbulence2NoTime(double x, double y, int octaves, double mult, double w, double h)
{
    double l_noise =  (
            (    getSTurbulence2(x/w*mult,y/h*mult,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getSTurbulence2((x-w)/w*mult,y/h*mult,octaves)
                    * (x)
                    * (h - y) )
            + (    getSTurbulence2((x-w)/w*mult,(y-h)/h*mult,octaves)
                    * (x)
                    * (y) )
            + (    getSTurbulence2(x/w*mult,(y-h)/h*mult,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

double tiledSTurbulence2(double x, double y, int octaves, double mult, double w, double h, double time)
{
    if (time==0) return tiledSTurbulence2NoTime(x,y,octaves,mult,w,h);
    double l_noise =  (
            (    getSTurbulence3(x/w*mult,y/h*mult,time,octaves)
                    * (w - x)
                    * (h - y) )
            + (    getSTurbulence3((x-w)/w*mult,y/h*mult,time,octaves)
                    * (x)
                    * (h - y) )
            + (    getSTurbulence3((x-w)/w*mult,(y-h)/h*mult,time,octaves)
                    * (x)
                    * (y) )
            + (    getSTurbulence3(x/w*mult,(y-h)/h*mult,time,octaves)
                    * (w - x)
                    * (y) )
    ) / (w*h);
    return l_noise;
}

//
// 3D
//
double tiledSTurbulence3NoTime(double x, double y, double z, int octaves, double mult, double w, double h, double d)
{
    double l_noise = (
            (    getSTurbulence3(x/w*mult,y/h*mult,z/d*mult,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z) )
            + (    getSTurbulence3((x-w)/w*mult,y/h*mult,z/d*mult,octaves)
                * (x)
                * (h - y)
                * (d - z) )
            + (    getSTurbulence3((x-w)/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (x)
                * (y)
                * (d - z) )
            + (    getSTurbulence3(x/w*mult,(y-h)/h*mult,z/d*mult,octaves)
                * (w - x)
                * (y)
                * (d - z) )


            + (    getSTurbulence3(x/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                * (w - x)
                * (h - y)
                * (z) )
            + (    getSTurbulence3((x-w)/w*mult,y/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (h - y)
                    * (z) )
            + (    getSTurbulence3((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (x)
                    * (y)
                    * (z) )
            + (    getSTurbulence3(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,octaves)
                    * (w - x)
                    * (y)
                    * (z) )

            ) / (w*h*d) ;
    return l_noise;
}

double tiledSTurbulence3(double x, double y, double z, int octaves, double mult, double w, double h, double d, double time)
{
    if (time == 0) return tiledSTurbulence3NoTime(x,y,z,octaves,mult,w,h,d);
    double l_noise = (
            (    getSNoise4(x/w*mult,y/h*mult,z/d*mult,time,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z) )
            + (    getSNoise4((x-w)/w*mult,y/h*mult,z/d*mult,time,octaves)
                * (x)
                * (h - y)
                * (d - z) )
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,z/d*mult,time,octaves)
                * (x)
                * (y)
                * (d - z) )
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,z/d*mult,time,octaves)
                * (w - x)
                * (y)
                * (d - z) )


            + (    getSNoise4(x/w*mult,y/h*mult,(z-d)/d*mult,time,octaves)
                * (w - x)
                * (h - y)
                * (z) )
            + (    getSNoise4((x-w)/w*mult,y/h*mult,(z-d)/d*mult,time,octaves)
                    * (x)
                    * (h - y)
                    * (z) )
            + (    getSNoise4((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,time,octaves)
                    * (x)
                    * (y)
                    * (z) )
            + (    getSNoise4(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,time,octaves)
                    * (w - x)
                    * (y)
                    * (z) )

            ) / (w*h*d) ;
    return l_noise;
}

//
// 4D
//
double tiledSTurbulence4(double x, double y, double z, double ww, int octaves, double mult, double w, double h, double d, double t)
{
    double l_noise = (
            (    getSTurbulence4(x/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                 * (w - x)
                 * (h - y)
                 * (d - z)
                 * (t - ww))
            + (    getSTurbulence4((x-w)/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (d - z)
                * (t - ww))
            + (    getSTurbulence4((x-w)/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (d - z)
                * (t - ww))
            + (    getSTurbulence4(x/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (d - z)
                * (t - ww))


            + (    getSTurbulence4(x/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (h - y)
                * (z)
                * (t - ww))
            + (    getSTurbulence4((x-w)/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (z)
                * (t - ww))
            + (    getSTurbulence4((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (z)
                * (t - ww))
            + (    getSTurbulence4(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (z)
                * (t - ww))
                    
            + (  getSTurbulence4(x/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (h - y)
                * (d - z)
                * (ww))
            + (    getSTurbulence4((x-w)/w*mult,y/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (d - z)
                * (ww))
            + (    getSTurbulence4((x-w)/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (d - z)
                * (ww))
            + (    getSTurbulence4(x/w*mult,(y-h)/h*mult,z/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (d - z)
                * (ww))


            + (    getSTurbulence4(x/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (h - y)
                * (z)
                * (ww))
            + (    getSTurbulence4((x-w)/w*mult,y/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (h - y)
                * (z)
                * (ww))
            + (    getSTurbulence4((x-w)/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (x)
                * (y)
                * (z)
                * (ww))
            + (    getSTurbulence4(x/w*mult,(y-h)/h*mult,(z-d)/d*mult,ww/t*mult,octaves)
                * (w - x)
                * (y)
                * (z)
                * (ww))

            ) / (w*h*d*t) ;
    return l_noise;
}


}
