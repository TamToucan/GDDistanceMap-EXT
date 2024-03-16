#ifndef UTIL_NAME_PTR_STORE_ITERATOR_H
#define UTIL_NAME_PTR_STORE_ITERATOR_H

#include <iterator>

#include "I_Iterator.h"

namespace Util {

    template<typename OBJTYPE> class NamePtrStore;

    //
    // Iterator for a NamePtrStore
    //
template<typename OBJTYPE>
class NamePtrStoreIterator : I_Iterator< std::pair<std::string, OBJTYPE*> >
{
public:
    NamePtrStoreIterator<OBJTYPE>(const NamePtrStore<OBJTYPE>* pStore) : m_pStore(pStore) { first(); }
    virtual ~NamePtrStoreIterator<OBJTYPE>() { }
    virtual void first()              { m_itr = m_pStore->m_store.begin(); }
    virtual void next()               { if (!isDone()) ++m_itr; }
    virtual bool isDone() const       { return (m_itr == m_pStore->m_store.end()); }
    virtual std::pair<std::string, OBJTYPE*> currentItem() const {
        return (isDone()) ? std::make_pair(std::string(""),m_pStore->m_notFoundT2)
                          : std::make_pair(m_itr->first, m_itr->second); }
protected:
    const NamePtrStore<OBJTYPE>*                                  m_pStore;
    typename NamePtrStore<OBJTYPE>::BasicStoreMap::const_iterator m_itr;

#ifdef CAN_DO_CIRCULAR_REFERENCE
    //
    // Want to return a new'ed Iterator from getIterator, but don't want to
    // have to remember to delete it. So just wrap in a crappy auto_ptr
public:
    typedef std::auto_ptr< NamePtrStoreIterator<OBJTYPE> > Ptr;
#endif
};

#ifndef CAN_DO_CIRCULAR_REFERENCE
template<typename OBJTYPE>
struct NamePtrStoreIteratorPtr
{
    typedef std::unique_ptr< NamePtrStoreIterator<OBJTYPE> > Type;
};
#endif

} // namespace

#endif
