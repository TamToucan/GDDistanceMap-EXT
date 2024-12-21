#ifndef UTIL_BASIC_HASH_TABLE_ITERATOR_H
#define UTIL_BASIC_HASH_TABLE_ITERATOR_H

// --------------------------------------------------------------------------
// Filename    : BasicHashTableIterator.h
// Project     : Util
// Description : Hash Table Iterator Definition
//
// A really crap iterator for use with the BasicHashTable.
// The interface doesn't follow "standard" iteartors. Instead
// the () operator is used to get the current object and move the
// iteartor onto the next.
// For example
//
//   BasicHashTableIterator<const TestObject> itr(hashTable);
//   const TestObject* pObj = itr();
//   while (pObj)
//   {
//      ...do something with pObj...
//      pObj = itr();
//   }
// --------------------------------------------------------------------------

#ifdef UTIL_HASH_DEBUG
#include <vector>
#endif

#include "BasicHashTable.h"

namespace Util {
template<class HashedObject> class BasicHashTable;

template< class HashedObject >
class BasicHashTableIterator
{
public:
    /**
     * @purpose: Construct an iterator to iterator over the given BasicHashTable
     */
    BasicHashTableIterator(BasicHashTable<HashedObject>& hashTable);

    /**
     * @purpose: Destructor
     */
    virtual ~BasicHashTableIterator();

    /**
     * @purpose: Get the current item and move on to the next
     */
    virtual HashedObject* operator()();

    /**
     * @purpose: Reset the iterator to start at the beginning
     */
    virtual void reset();

    /**
     * @purpose: Find the given object and initialise the iterator
     * to return the next object. Used if want to start iterating
     * from a given point
     */
    virtual HashedObject* findAndInit(const HashedObject* pItem);

private:
    BasicHashTable<HashedObject>&                m_hashTable;
    unsigned long                                m_curBucket;
    //typename BasicHashTable<HashedObject>::Link* m_pCurLink;
    typename BasicHashTable<HashedObject>::LinkClass::Link* m_pCurLink;
};

} // namespace

#include "BasicHashTableIterator.I"

#endif

