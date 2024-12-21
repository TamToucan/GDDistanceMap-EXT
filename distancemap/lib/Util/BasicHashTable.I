
// --------------------------------------------------------------------------
// Filename    : BasicHashTable.I
// Project     : Util
// Description : Hash Table Implementation
// --------------------------------------------------------------------------

#include <string.h>
#include "BasicHashTable.h"
#include "I_HashTableObject.h"

#ifdef UTIL_HASH_DEBUG
#include <vector>
#include <iterator>
#include <sstream>
#endif

namespace Util {

/******************************************************************************/

/////////////////////////////////////////////////////////
// CONSTRUCTOR
//
template<class HashedObject>
BasicHashTable<HashedObject>::BasicHashTable(long tableSize) :
    m_tableSize(tableSize),
    m_numEntries(0)
{
    //
    // Create the hash table array of Link objects
    // - NOTE: These are the head of the linked list
    //   and are not used themselves i.e. they are
    //   not allocated using getNewLink
    //
    m_pTheTable = new HashLink[m_tableSize];
    memset(m_pTheTable, 0x00, sizeof(HashLink)*m_tableSize);
#ifdef UTIL_HASH_DEBUG
    LinkClass::instance()->m_allBasicHashTables.push_back(this);
#endif
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// DESTRUCTOR
//
template<class HashedObject>
BasicHashTable<HashedObject>::~BasicHashTable()
{
    clear();
    delete[] m_pTheTable;
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// CLEAR TABLE
//
// Remove all the objects from the hash table
//
template<class HashedObject>
void BasicHashTable<HashedObject>::clearTable(bool deleteObjectsFlag)
{
    //
    // Go through each of the hash table buckets
    //
    for (unsigned long bucket=0; bucket < m_tableSize; ++bucket)
    {
        //
        // Remove each link in the bucket's links list
        // - Don't delete the bucket link as that is the head
        //   of the list which is not part of it i.e. it was
        //   not allocated using getNewLink
        //
        HashLink* l_pCurLink;
        HashLink* l_pNxtLink = m_pTheTable[bucket].nextPtr;

        m_pTheTable[bucket].nextPtr = 0;
        while (0 != (l_pCurLink=l_pNxtLink))
        {
            if (deleteObjectsFlag)
            {
                delete (l_pCurLink->itemPtr);
            }
            l_pNxtLink = l_pCurLink->nextPtr;
            LinkClass::instance()->deleteLink(l_pCurLink);
            --m_numEntries;
        }
    }
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// CLEAR
//
// Remove all the objects from the hash table
//
template<class HashedObject>
void BasicHashTable<HashedObject>::clear()
{
#ifdef UTIL_HASH_DEBUG
    LinkClass::instance()->m_pCurTable = this;
#endif
    clearTable(false);
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// CLEAR AND DESTORY
//
// Removed and Deletes all the objects from the hash table
//
template<class HashedObject>
void BasicHashTable<HashedObject>::clearAndDestroy()
{
#ifdef UTIL_HASH_DEBUG
    LinkClass::instance()->m_pCurTable = this;
#endif
    clearTable(true);
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// FIND
//
// Find the object in the Hash Table
// - Return 0 if not found
//
template<class HashedObject>
HashedObject* BasicHashTable<HashedObject>::find(const HashedObject* pItem) const
{
#ifdef UTIL_HASH_DEBUG
    LinkClass::instance()->m_pCurTable = this;
#endif
    HashLink* l_pItemLink = LinkClass::instance()->getLinkBefore(chainHead(pItem), pItem)->nextPtr;

    if (0 == l_pItemLink)
    {
        return 0;
    }
    else
    {
        return (l_pItemLink->itemPtr);
    }
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// INSERT
//
// Insert the object into the Hash Table
//
template<class HashedObject>
HashedObject* BasicHashTable<HashedObject>::insert(HashedObject* pItem)
{
#ifdef UTIL_HASH_DEBUG
    LinkClass::instance()->m_pCurTable = this;
#endif
    //
    // Get a new Link and store a pointer to the item
    //
    HashLink* l_pItemLink = LinkClass::instance()->getNewLink();
    l_pItemLink->itemPtr = pItem;
    ++m_numEntries;
    //
    // Point the new link at the start of the list
    // i.e. chainHead->nextPtr then point the start
    // of the list at the newLink
    //
    HashLink* chainPtr = chainHead(pItem);
    l_pItemLink->nextPtr = chainPtr->nextPtr;
    chainPtr->nextPtr = l_pItemLink;

    return pItem;
}

/******************************************************************************/

/////////////////////////////////////////////////////////
// REMOVE
//
// Remove the item from the Hash Table
// - Returns the removed item or 0 if not found
//
template<class HashedObject>
HashedObject* BasicHashTable<HashedObject>::remove(HashedObject* pItem)
{
#ifdef UTIL_HASH_DEBUG
    LinkClass::instance()->m_pCurTable = this;
#endif
    //
    // Get the link of the item and the link before it
    //
    HashLink* l_pBeforeLink = LinkClass::instance()->getLinkBefore(chainHead(pItem), pItem);
    HashLink* l_pItemLink = l_pBeforeLink->nextPtr;
    HashedObject* l_retItem = 0;

    //
    // If item found then set the before link to the
    // the link after the item and delete the items link
    //
    if (0 != l_pItemLink)
    {
        l_retItem = (l_pItemLink->itemPtr);
        l_pBeforeLink->nextPtr = l_pItemLink->nextPtr;
        LinkClass::instance()->deleteLink(l_pItemLink);
        --m_numEntries;
    }
    return l_retItem;
}

/******************************************************************************/

/******************************************************************************/

template<class HashedObject>
BasicHashLink<HashedObject>* BasicHashLink<HashedObject>::instance()
{
    static BasicHashLink<HashedObject> l_instance;
    return &l_instance;
}

template<class HashedObject>
BasicHashLink<HashedObject>::BasicHashLink()
{
    m_pFreeLinks = 0;
    m_pAllocs = 0;
    m_linkBlockSize = 1;
#ifdef UTIL_HASH_DEBUG
    m_totalCreatedLinks = 0;
    m_numFreeLinks = 0;
    m_pCurTable = 0;
#endif
}

template<class HashedObject>
BasicHashLink<HashedObject>::~BasicHashLink()
{
    // Delete all the blocks of Links allocated
    while (m_pAllocs)
    {
       LinkAlloc* pNext = m_pAllocs->nextPtr;
       delete[] m_pAllocs->blockPtr;
       delete m_pAllocs;
       m_pAllocs = pNext;
    }
}

//
// Get the ptr to the Link before the item
// - If not found then the nextPtr will be 0
//
template<class HashedObject>
typename BasicHashLink<HashedObject>::Link*
BasicHashLink<HashedObject>::getLinkBefore(Link* pLinkBefore,
                                           const HashedObject* pItem) const
{
#ifdef UTIL_HASH_DEBUG
    if (0 != pLinkBefore->itemPtr)
    {
        hashBug("getLinkBefore");
    }
#endif
    while (  (0 != pLinkBefore->nextPtr)
            && (pItem->isEqual(pLinkBefore->nextPtr->itemPtr) == false) )
    {
        pLinkBefore = pLinkBefore->nextPtr;
    }
    return pLinkBefore;
}


/******************************************************************************/


//
// Get a new link object from the m_pFreeLinks list
//
template<class HashedObject>
typename BasicHashLink<HashedObject>::Link* BasicHashLink<HashedObject>::getNewLink()
{
    // If there are no free links then need to allocate a new block
    if (0 == m_pFreeLinks)
    {
        initFreeLinksBlock();
    }
    //
    // Return the start of the free list and update
    // the free list to the next link
    //
#ifdef UTIL_HASH_DEBUG
    if ((0 == m_numFreeLinks) or (0 == m_pFreeLinks))
    {
        hashBug("getNewLink");
    }
    --m_numFreeLinks;
#endif
    Link* newLink = m_pFreeLinks;
    m_pFreeLinks = m_pFreeLinks->nextPtr;
    return newLink;
}

/******************************************************************************/

// Delete a link object and return it to the m_pFreeLinks list
//
template<class HashedObject>
void BasicHashLink<HashedObject>::deleteLink(typename BasicHashLink<HashedObject>::Link* pLink)
{
    //
    // Point the deleted link at the start of the free list
    // and set the start of the free list to the deleted link
    //
    pLink->itemPtr = 0;
    pLink->nextPtr = m_pFreeLinks;
    m_pFreeLinks = pLink;
#ifdef UTIL_HASH_DEBUG
    ++m_numFreeLinks;
    if ((m_numFreeLinks > m_totalCreatedLinks) or (0 == m_pFreeLinks))
    {
        hashBug("deleteLink");
    }
#endif
}

/******************************************************************************/

template<class HashedObject>
long BasicHashLink<HashedObject>::setBlockSize(long linkBlockSize, bool allowDecreaseSize)
{
    long l_prevSize = m_linkBlockSize;

    // Change the value if non-zero and not equal to the current value
    // Allow the value to be decreased if allowDecreaseSize is true
    // - This is incase there are multiple callers the probably want to
    //   use the largest
    if (    (0 != linkBlockSize)
        and (linkBlockSize != m_linkBlockSize)
        and ((linkBlockSize > l_prevSize) or (allowDecreaseSize)) )
    {
        m_linkBlockSize = linkBlockSize;
        // If there are no free links then allocate more using
        // the new block size.
        // NOTE: This means if there are multiple callers the first
        // set of links is allocated using the first size
        // But this is probably better than doing the first allocation
        // when the first object is added to the first BasicHashTable
        if (0 == m_pFreeLinks)
        {
            initFreeLinksBlock();
        }
    }
    return l_prevSize;
}

/******************************************************************************/
//
// Initialise a block of links and store them in the m_pFreeLinks list
// The number of links in the block is in m_linkBlockSize
//
template<class HashedObject>
void BasicHashLink<HashedObject>::initFreeLinksBlock()
{
    //
    // Create an array of Link objects
    //
    m_pFreeLinks = new Link[m_linkBlockSize];

    // Remember this allocation block of links
    LinkAlloc* l_pNext = m_pAllocs;
    m_pAllocs = new LinkAlloc;
    m_pAllocs->blockPtr = m_pFreeLinks;
    m_pAllocs->nextPtr = l_pNext;

    // Chain all the nextPtrs of the array of link objects together
    for (long l_idx=0; l_idx < m_linkBlockSize-1; ++l_idx)
    {
        m_pFreeLinks[l_idx].nextPtr = &m_pFreeLinks[l_idx+1];
        m_pFreeLinks[l_idx].itemPtr = 0;
    }
    m_pFreeLinks[m_linkBlockSize-1].nextPtr = 0;
    m_pFreeLinks[m_linkBlockSize-1].itemPtr = 0;
#ifdef UTIL_HASH_DEBUG
    m_totalCreatedLinks += m_linkBlockSize;
    m_numFreeLinks = m_linkBlockSize;
#endif
}

/******************************************************************************/

#ifdef UTIL_HASH_DEBUG
/////////////////////////////////////////////////////////
// DEBUG ROUTINES
#define UTIL_LOG_ERROR(param) \
do { \
    std::cerr << "ERROR:" << param << std::endl; \
} while(0)

#define UTIL_LOG_DEBUG(param) \
do { \
    std::cerr << "DEBUG:" << param << std::endl; \
} while(0)


template<class HashedObject>
void BasicHashLink<HashedObject>::hashBug(const char* str) const
{
    UTIL_LOG_ERROR("### HashBug:: " << str);
    UTIL_LOG_ERROR("numFreeLinks=" << m_numFreeLinks << " maxLinks=" << m_linkBlockSize
                  << " m_pFreeLinksPtr=" << m_pFreeLinks
                  << " &m_pFreeLinksPtr=" << &m_pFreeLinks);

    UTIL_LOG_ERROR("FREE LINKS DUMP");
    Link* l_pLink = m_pFreeLinks;
    while (0 != l_pLink)
    {
        UTIL_LOG_ERROR("  " << l_pLink->itemPtr);
        l_pLink = l_pLink->nextPtr;
    }

    UTIL_LOG_ERROR("###############\nCurrent Hash table\n");
    m_pCurTable->dump();

    typename std::vector<const BasicHashTable<HashedObject>* >::const_iterator l_itr;
    l_itr = m_allBasicHashTables.begin();
    int i =0;
    while (l_itr != m_allBasicHashTables.end())
    {
        ++i;
        if ((*l_itr) != m_pCurTable)
        {
            UTIL_LOG_ERROR("###############\nHASH TABLE " << i);
            (*l_itr)->dump();
        }
        ++l_itr;
    }
    abort();
}


template<class HashedObject>
void BasicHashTable<HashedObject>::dump() const
{
    UTIL_LOG_DEBUG("HASH TABLE DUMP");
    UTIL_LOG_DEBUG("numEntries   " << m_numEntries);
    int i=0;
    HashLink* l_pLink = &m_pTheTable[i];

    std::ostringstream* l_pOStrDump = new std::ostringstream;
    *l_pOStrDump << "BUCKET[" << i << "]: ";

    do {
        if (0 == (l_pLink = l_pLink->nextPtr))
        {
            ++i;
            while (    (i < m_tableSize)
                   and (0 == (l_pLink=m_pTheTable[i].nextPtr)) )
            {
                ++i;
            }
            if (i == m_tableSize)
            {
                l_pLink = 0;
            }
            else
            {
                UTIL_LOG_DEBUG( l_pOStrDump->str());
                delete l_pOStrDump;
                l_pOStrDump = new std::ostringstream;
                *l_pOStrDump << "BUCKET[" << i << "]: ";
            }
        }
        if (0 != l_pLink)
        {
            *l_pOStrDump << (void*) l_pLink->itemPtr << "  ";
        }
    } while (0 != l_pLink);
    UTIL_LOG_DEBUG(l_pOStrDump->str());
    delete l_pOStrDump;
}
#endif


/******************************************************************************/

} // namespace

