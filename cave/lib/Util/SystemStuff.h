/*
 * SystemStuff.h
 *
 *  Created on: 18-Aug-2010
 *      Author: Tam
 */

#ifndef UTIL_SYSTEMSTUFF_H_
#define UTIL_SYSTEMSTUFF_H_

// Should check if it's GCC I suppose
#ifndef NEVER_INLINE
#define NEVER_INLINE __attribute__((__noinline__))
#endif

#define COMPILE_TIME_SIZE_CHECK(name, size) \
    typedef name SizeCheck_ ## name[ ((size==sizeof(name))*2) -1 ]

// Don't have a define for this yet, but put here
// to show how the code would look
#if defined(CPP_TR1)
#include <cstdint>

namespace Sys {

typedef std::tr1::int8_t int8;
typedef std::tr1::uint8_t uint8;
typedef std::tr1::int16_t int16;
typedef std::tr1::uint16_t uint16;
typedef std::tr1::int32_t int32;
typedef std::tr1::uint32_t uint32;
typedef float float32;

} // namespace

#else

#include <stdint.h>

namespace Sys {

typedef int8_t int8;
typedef uint8_t uint8;
typedef int16_t int16;
typedef uint16_t uint16;
typedef int32_t int32;
typedef uint32_t uint32;
typedef float float32;

} // namespace

#endif

#endif /* UTIL_SYSTEMSTUFF_H_ */
