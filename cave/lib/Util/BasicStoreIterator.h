#ifndef UTIL_BASIC_STORE_ITERATOR_H
#define UTIL_BASIC_STORE_ITERATOR_H

#include <memory>
#include <iterator>

#include "I_Iterator.h"

namespace Util {

    template <typename T1,typename T2> class BasicStore;

    //
    // Iterator for a BasicStore
    //
template<typename T1, typename T2>
class BasicStoreIterator : I_Iterator<T2>
{
    //BasicStoreIterator<T1,T2>(const BasicStoreIterator& i) : m_pStore(i.m_pStore) { first(); }
public:
    BasicStoreIterator<T1,T2>(const BasicStore<T1, T2>* pStore) : m_pStore(pStore) { first(); }
    virtual ~BasicStoreIterator<T1,T2>() { }
    virtual void first()              { m_itr = m_pStore->m_store.begin(); }
    virtual void next()               { if (!isDone()) ++m_itr; }
    virtual bool isDone() const       { return (m_itr == m_pStore->m_store.end()); }
    virtual T1 currentKey() const     { return m_itr->first; } // EEK! Can't return a NOT FOUND!
    virtual T2 currentItem() const    { return (isDone()) ? m_pStore->m_notFoundT2 : m_itr->second; }
    virtual T2 curItemNoCheck() const { return m_itr->second; }
protected:
    const BasicStore<T1,T2>*  m_pStore;
    typename BasicStore<T1,T2>::BasicStoreMap::const_iterator m_itr;

    // See BasicStore.h
#ifdef CAN_DO_CIRCULAR_REFERENCE
public:
    // Want to return a new'ed Iterator from getIterator, but don't want to
    // have to remember to delete it. So just wrap in a crappy auto_ptr
    typedef std::auto_ptr< BasicStoreIterator<T1,T2> > Ptr;
#endif
};

#ifndef CAN_DO_CIRCULAR_REFERENCE
template<typename T1, typename T2>
struct BasicStoreIteratorPtr
{
    typedef std::unique_ptr< BasicStoreIterator<T1,T2> > Type;
};
#endif

//
// HELPER METHOD
//
template <typename T1, typename T2>
inline void basicStoreDeleteAndClear(BasicStore<T1, T2>& bs)
{
#ifdef CAN_DO_CIRCULAR_REFERENCE
    for (typename BasicStoreIterator<T1, T2>::Ptr l_itr = bs.getIterator();
#else
    for (typename BasicStoreIteratorPtr<T1, T2>::Type l_itr = bs.getIterator();
#endif
         not l_itr->isDone();
         l_itr->next())
    {
        delete l_itr->currentItem();
    }
    bs.BasicStore<T1, T2>::clear();
}

} // namespace

#endif
