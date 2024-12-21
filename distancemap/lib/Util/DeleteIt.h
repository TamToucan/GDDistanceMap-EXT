#ifndef UTIL_DELETE_IT_H
#define UTIL_DELETE_IT_H

#include <algorithm>
#include <functional>
#include "STLStuff.h"

namespace Util {

//
// Functor to delete object
//
template <typename DeletablePtr>
struct DeleteIt : public std::unary_function<DeletablePtr, void>
{
public:
    void operator () (DeletablePtr const& obj) const { delete obj; }
};

//
// Helper function to delete all the objects in a container and clear it
//
template <typename Container>
inline void deleteAndClear(Container& container)
{
    std::for_each(container.begin(), container.end(), DeleteIt<typename Container::value_type>());
    container.clear();
}

//
// Helper function to delete the .first object of a container of std::pair's and clear it
//
template <typename Container>
inline void deleteFirstAndClear(Container& c)
{
    std::for_each(c.begin(), c.end(), compose1(DeleteIt<typename Container::value_type::first_type>(),
                                               Util::select1st<typename Container::value_type>()) );
    c.clear();
}

//
// Helper function to delete the both objects of container of std::pair's and clear it
//
template <typename Container>
inline void deleteBothAndClear(Container& c)
{
    std::for_each(c.begin(), c.end(), compose1(DeleteIt<typename Container::value_type::first_type>(),
                                               Util::select1st<typename Container::value_type>()) );
    std::for_each(c.begin(), c.end(), compose1(DeleteIt<typename Container::value_type::second_type>(),
                                               Util::select2nd<typename Container::value_type>()) );
    c.clear();
}

//
// Helper function to delete the .first object of a container of std::pair's
// and clear it
//
template <typename Container>
inline void deleteSecondAndClear(Container& c)
{
    std::for_each(c.begin(), c.end(), compose1(DeleteIt<typename Container::value_type::second_type>(),
                                               Util::select2nd<typename Container::value_type>()) );
    c.clear();
}

//
// Helper function to delete and 0 a pointer
//
template <typename Ptr>
inline void deleteAndNull(Ptr& ptr)
{
    delete ptr;
    ptr = 0;
}

//
// Find and Delete
// 
template <typename Container>
inline void findAndDelete(Container& container, typename Container::value_type p)
{
    typename Container::iterator l_found = std::find(container.begin(), container.end(), p);
    if (l_found != container.end())
    {
        delete *l_found;
        container.erase(l_found);
    }
}

} // namespace

#endif

