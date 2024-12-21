// --------------------------------------------------------------------------
// Filename    : BasicHashTableIterator.I
// Project     : Util
// Description : Hash Table Iterator Implementation
// --------------------------------------------------------------------------

#include <string.h>
#include "BasicHashTableIterator.h"
#include "BasicHashTable.h"

namespace Util {

/******************************************************************************/

/////////////////////////////////////////////////////////
// CONSTRUCTOR
//
template<class HashedObject>
BasicHashTableIterator<HashedObject>::BasicHashTableIterator(BasicHashTable<HashedObject>& hashTable) :
    m_hashTable(hashTable)
{
    reset();
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// DESTRUCTOR
//
template<class HashedObject>
BasicHashTableIterator<HashedObject>::~BasicHashTableIterator()
{
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// OPERATOR()
//
template<class HashedObject>
HashedObject* BasicHashTableIterator<HashedObject>::operator()()
{
    if (0 == m_pCurLink)
    {
        return 0;
    }
    //
    // Move to the next link in the chain. If 0 then
    // move the next bucket
    //
    if (0 == (m_pCurLink = m_pCurLink->nextPtr))
    {
        //
        // Keep moving onto the next bucket until find a non-empty one
        //
        ++m_curBucket;
        while (    (m_curBucket < m_hashTable.m_tableSize)
               and (0 == (m_pCurLink=m_hashTable.m_pTheTable[m_curBucket].nextPtr)) )
        {
            ++m_curBucket;
        }
        //
        // If reached last bucket then set m_pCurLink to 0. Will stay 0
        // until reset is called
        //
        if (m_curBucket == m_hashTable.m_tableSize)
        {
            m_pCurLink = 0;
            return 0;
        }
    }
    return (m_pCurLink->itemPtr);
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// RESET
//
template<class HashedObject>
void BasicHashTableIterator<HashedObject>::reset()
{
    m_curBucket = 0;
    m_pCurLink = &m_hashTable.m_pTheTable[m_curBucket];
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// findAndInit
// Tries to find the supplied pItem in the BasicHashTable; if it
// is found then the iterator is initialised on that position and
// the object is returned, otherwise 0 is returned, in which case
// the iterator is reset.
// Either way, the iterator is usable.
//
template<class HashedObject>
HashedObject* BasicHashTableIterator<HashedObject>::findAndInit(const HashedObject* pItem)
{
    HashedObject* l_retItem = 0;
    if ( pItem )
    {
        typename BasicHashTable<HashedObject>::LinkClass::Link* l_pItemLink =
            BasicHashTable<HashedObject>::LinkClass::instance()->
                   getLinkBefore(m_hashTable.chainHead(pItem), pItem)->nextPtr;

        if ( l_pItemLink )
        {
            //
            //Calculate what m_pCurLink and m_curBucket should be...
            //
            m_pCurLink = l_pItemLink;

            typename BasicHashTable<HashedObject>::LinkClass::Link* pHead = m_hashTable.chainHead(pItem);
            m_curBucket = pHead - (m_hashTable.m_pTheTable);
            l_retItem = (l_pItemLink->itemPtr);
        }
    }
    if ( 0 == l_retItem )
    {
        reset();
    }
    return l_retItem;
}

/******************************************************************************/

} // namespace

