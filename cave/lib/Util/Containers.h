#ifndef UTIL_CONTAINERS_H
#define UTIL_CONTAINERS_H
#include <vector>
#include <utility>
#include <set>
#include <iterator>
#include <algorithm>
#include <string>

namespace Util {

typedef std::vector<std::string> StringVector;
typedef std::pair<std::string, std::string> StringPair;
typedef std::vector< StringPair > StringPairVector;

typedef std::set<std::string> UniqStrings;

//
// Return index of the iterator from the start of vector
template<typename T>
inline int
itrToIndex(const T& vec, const typename T::const_iterator& itr)
{
    return std::distance<typename T::const_iterator>(vec.begin(), itr);
}

//
// Return iterator at +index from start or -index from end i.e. -1 = last item
// Const version
//
template<typename T>
typename T::const_iterator indexToItr(const T& list, int index)
{
    typename T::const_iterator l_itr = (index < 0) ? list.end() : list.begin();
    std::advance<typename T::const_iterator>(l_itr, index);
    return l_itr;
}

//
// Return iterator at +index from start or -index from end i.e. -1 = last item
// Non-Const version
//
template<typename T>
typename T::iterator indexToItr(T& list, int index)
{
    typename T::iterator l_itr = (index < 0) ? list.end() : list.begin();
    std::advance(l_itr, index);
    return l_itr;
}

//
// Find the index of the item in the container
// T2 must support std::find and std::distance e.g. std::vector
//
template<typename T1, typename T2>
inline int
indexOf( const T1& item, const T2& vec)
{
    typename T2::const_iterator l_itr = std::find(vec.begin(), vec.end(), item);
    return (l_itr != vec.end())
           ? itrToIndex(vec, l_itr)
           : -1;
}

//
// Find the index of the item in the container and erase it
// T2 must support std::find and std::distance e.g. std::vector
//
template<typename T1, typename T2>
inline bool
findAndErase( T1& item, T2& vec)
{
    typename T2::iterator l_itr = std::find(vec.begin(), vec.end(), item);
    bool l_retVal = (l_itr != vec.end());
    if (l_retVal)	{
        vec.erase(l_itr);
    }
    return l_retVal;
}


template <typename Container>
void append(Container& to, const Container& from)
{
    std::copy(from.begin(), from.end(), std::back_inserter(to));
}

} // namespace

#endif
