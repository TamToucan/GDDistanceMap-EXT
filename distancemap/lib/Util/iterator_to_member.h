#ifndef UTIL_ITER_TO_MEM_H
#define UTIL_ITER_TO_MEM_H

#include <iterator>

namespace Util {

    //
    // A template class to provide an iterator to a member
    // of the data pointed to by the iterator.
    // Copied from Dr Dobbs
    //

template<
  typename ITR,  // type of original iterator
  typename TYP,  // type pointed to by original iterator
  typename DATA  // type of the member we want to point to
>
class iterator_to_member
{
public:
    //
    // Some typedefs
    //
    typedef typename std::iterator_traits<ITR>::iterator_category iterator_category;
    typedef typename std::iterator_traits<ITR>::difference_type   difference_type;
    typedef DATA value_type;
    typedef DATA* pointer;
    typedef DATA& reference;

    //
    // Construction from an iterator and a pointer to member.
    //
    iterator_to_member(ITR from, DATA TYP::* memptr) : m_it(from), m_memptr(memptr)
    {
    }

    //
    // These access the interator then access the data via the member pointer
    //
    reference operator*() const { return ((*m_it).*m_memptr); }
    pointer operator->() const { return &((*m_it).*m_memptr); }
    reference operator[](difference_type n) const { return m_it[n].*m_memptr; }

    //
    // Position operators just go to member iterator
    //
    iterator_to_member& operator++() { ++m_it; return *this; }
    iterator_to_member& operator--() { --m_it; return *this; }

    iterator_to_member operator++(int) { m_it++; return *this; }
    iterator_to_member operator--(int) { m_it--; return *this; }

    iterator_to_member& operator+=(difference_type n) { m_it += n; return *this; }
    iterator_to_member& operator-=(difference_type n) { m_it -= n; return *this; }

    iterator_to_member operator+(difference_type n) const { return iterator_to_member(m_it+n, m_memptr); }
    iterator_to_member operator-(difference_type n) const { return iterator_to_member(m_it-n, m_memptr); }

    bool operator==(const iterator_to_member<ITR, TYP, DATA>& rhs) const { return m_it == rhs.m_it; }
    bool operator!=(const iterator_to_member<ITR, TYP, DATA>& rhs) const { return m_it != rhs.m_it; }

protected:
    ITR m_it;
    value_type TYP::* m_memptr;
};

//
// A Make helper function
//
template<typename ITR, typename TYP, typename DATA>
iterator_to_member<ITR, TYP, DATA>
make_iterator_to_member(ITR it, DATA TYP::* memptr)
{
    return iterator_to_member<ITR, TYP, DATA>(it, memptr);
}

} // namepace

#endif


