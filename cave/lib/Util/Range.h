#ifndef UTIL_RANGE_H
#define UTIL_RANGE_H

#include <iostream>

namespace Util {

    template<typename T>
    struct Range {
        T min;
        T max;
        Range() : min(0), max(0) { }
        Range(const T& mn, const T& mx)
            : min(mn), max(mx) { }
    };

    typedef Range<int> IntRange;
    typedef Range<float> FloatRange;


// Helper cout (for debug)
template<typename T>
std::ostream& operator<<(std::ostream& os, const Util::Range<T>& r) {
    return os << r.min << "..." << r.max;
}

} // namespace

#endif

