#ifndef UTIL_HASH_kEY_FUNCS_H
#define UTIL_HASH_kEY_FUNCS_H

#include <string>

namespace Util {

class HashKeyFuncs
{
public:
    static unsigned long stringHash(const std::string& rStr)
    {
        return hash(rStr.c_str(), rStr.length());
    }

    static unsigned long hash(const char* pByte, long numBytes)
    {
        long l_hash = 0;

        for (int i=0; i < numBytes; ++i)
        {
            l_hash = (l_hash << 4) + pByte[i];
            long l_val = l_hash & 0xF0000000L;
            if (l_val)
            {
                l_hash ^= (l_val >> 24);
                l_hash &= ~l_val;
            }
        }
        return l_hash;
    }
};

} // namespace

#endif

