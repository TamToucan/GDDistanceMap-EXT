#ifndef UTIL_SLEEP_H
#define UTIL_SLEEP_H

#ifdef WIN32
#  include <windows.h>
#else
#  include <unistd.h>
#endif

namespace Util {

inline void SleepSecs(unsigned int secs)
{
#ifdef WIN32
    Sleep(secs * 1000);
#else
    while (secs)
    {
        secs = sleep(secs);
    }
#endif
}

} // namespace
#endif

