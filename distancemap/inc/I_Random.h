#ifndef I_RANDOM_H
#define I_RANDOM_H

#include "../../cave/inc/Range.h"
#include "../../cave/inc/SystemStuff.h"

namespace RNG {

class I_Random
{
public:
    typedef Sys::int32 int32;
    typedef Sys::uint32 uint32;
public:
    I_Random()
    {
        // detect computer architecture
        union {
            double f;
            I_Random::uint32 i[2];
        } detectArch;
        detectArch.f = 1.0;
        // Note: Old versions of the Gnu g++ compiler may make an error here,
        // compile with the option  -fenum-int-equiv  to fix the problem
        if (detectArch.i[1] == 0x3FF00000)
        {
            m_architecture = LITTLE_ENDIAN1;
        }
        else
        if (detectArch.i[0] == 0x3FF00000)
        {
            m_architecture = BIG_ENDIAN1;
        }
        else
        {
            m_architecture = NONIEEE;
        }
    }

    // destructor
    virtual ~I_Random() { }

    // re-seed
    virtual void initialise(unsigned int seed) = 0;

    // seed by more than 32 bits
    virtual void initialise(unsigned int seeds[], int length) = 0;

    // output random float, 0...1
    virtual double getFloat() = 0;

    // get a random value between min, max
    template<typename TT>
    TT getRange(const TT& min, const TT& max) {
        return TT(double((max+1)-min) * getFloat() /*+ 0.5*/) + min;
    }

    // get a random integer between min, max
    int getInt(int min, int max)
    {
        return getRange<int>(min,max);
    }

    template<typename TT>
    TT getRange(const Util::Range<TT>& r) {
        return getRange<TT>(r.min, r.max);
    }

protected:
    double random2double(I_Random::uint32 r)
    {
        union {
            double f;
            I_Random::uint32 i[2];
        } convert;
        // The fastest way to convert random bits to floating point is as
        // follows: Set the binary exponent of a floating point number to
        // 1+bias and set the mantissa to random bits. This will give a random
        // number in the interval [1,2). Then subtract 1.0 to get a random
        // number in the interval [0,1).
        switch (m_architecture)
        {
        case LITTLE_ENDIAN1:
            convert.i[0] =  r << 20;
            convert.i[1] = (r >> 12) | 0x3FF00000;
            return convert.f - 1.0;
        case BIG_ENDIAN1:
            convert.i[1] =  r << 20;
            convert.i[0] = (r >> 12) | 0x3FF00000;
            return convert.f - 1.0;
        case NONIEEE:
        default:
            // This somewhat slower method works for all architectures,
            // including non-IEEE floating point representation:
            return (double)r * (1./((double)(I_Random::uint32)(-1L)+1.));
        }
    }

protected:
    enum TArch {
        LITTLE_ENDIAN1,
        BIG_ENDIAN1,
        NONIEEE
    };
    TArch m_architecture;
};


template<>
inline
float I_Random::getRange<float>(const float& min, const float& max) {
        return float((max-min) * getFloat()) + min;
}

template<>
inline
double I_Random::getRange<double>(const double& min, const double& max) {
    return ((max-min) * getFloat()) + min;
}

} // namespace

#endif

