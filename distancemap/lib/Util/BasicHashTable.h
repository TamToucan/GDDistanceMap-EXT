#ifndef UTIL_BASIC_HASH_TABLE_H
#define UTIL_BASIC_HASH_TABLE_H

// --------------------------------------------------------------------------
// Filename    : BasicHashTable.h
// Project     : Util
// Description : Hash Table Definition
//
// A very basic implementation of the I_HashTable using an array containing
// linked lists of objects chained via BasicHashTable::Link objects. The
// Link objects are allocated in blocks to increase performance. They are
// never deleted (instead being returnd to the list of availble Link objects).
// A single pool of objects is used by all BasicHashTable objects.
//
// NOTE: Currently only pointers to objects derived from I_HashTableObject
// can be used with this class. If my template skills improve this may change.
// --------------------------------------------------------------------------

#ifdef UTIL_HASH_DEBUG
#include <vector>
#endif

#include "I_HashTable.h"
#include "I_HashTableObject.h"

namespace Util {

template<class HashedObject> class BasicHashTableIterator;

template<class HashedObject> class BasicHashLink;

template<class HashedObject>
class BasicHashLink
{
private:
    BasicHashLink();

public:
    static BasicHashLink* instance();

    virtual ~BasicHashLink();

    /**
     * @purpose: Sets the block size that Links are allocated in.
     * These blocks are shared by ALL BasicHashTable objects to allow
     * more efficient use of memory. If called multiple times
     * then the largest size passed into this function is used unless
     * the allowDecreaseSize.
     * The change takes effect when the current set of blocks is used up.
     * Returns the previous value of blockSize
     * NOTE: This routine MUST be called before any objects are added
     * to a BasicHashTable.
     */
    long setBlockSize(long linkBlockSize, bool allowDecreaseSize=false);

    /**
     * @purpose: Structure to link objects in the Hash Table array
     * together as a linked list. Blocks of Link objects are created
     * which all BasicHashTable objects use to get new links from.
     */
    struct Link {
        HashedObject* itemPtr;
        struct Link* nextPtr;
    };

    /**
     * @purpose:  Get the ptr to the Link before the item
     *  - If not found then the nextPtr will be 0
     */
    Link* getLinkBefore(Link* pItemChainHead, const HashedObject* pItem) const;

    /**
     * @purpose: gets a new Link from the current block a Links i.e.
     * from pFreeLinks list. If the block is empty a new block is allocated
     */
    Link* getNewLink();

    /**
     * @purpose: free's a Link and returns it to the pFreeLinks list
     */
    void deleteLink(Link* pLink);


private:
    /**
     * @purpose: A linked list of the blocks of Link's allocated by this
     * class so it can free them on destruction
     */
    struct LinkAlloc {
        Link* blockPtr;
        struct LinkAlloc* nextPtr;
    };
    LinkAlloc* m_pAllocs;


    /**
     * @purpose: A linked list of s_Link objects that are used
     * (by ALL BasicHashTables) to insert objects together in the hash table.
     */
    Link* m_pFreeLinks;

    /**
     * @purpose: Defines the size of Blocks of Link arrays that are
     * allocated. Once all those Link objects are used i.e. that many
     * objects have been hashed IN TOTAL i.e. not just this BasicHashTable,
     * object but count all BasicHashTables, then a new m_linkBlockSize is
     * allocated.
     */
    long m_linkBlockSize;

    /**
     * @purpose: Allocates a new block of m_linkBlockSize Link objects
     * and puts them onto the m_pFreeLinks list
     */
    void initFreeLinksBlock();

#ifdef UTIL_HASH_DEBUG
public:
    /**
     * @purpose: Debug members and methods
     */
    long                                               m_totalCreatedLinks;

    long                                               m_numFreeLinks;

    const BasicHashTable<HashedObject>*                m_pCurTable;

    std::vector< const BasicHashTable<HashedObject>* > m_allBasicHashTables;

    void hashBug(const char* str) const;
#endif

};


template<class HashedObject>
class BasicHashTable : public I_HashTable<HashedObject>
{
    friend class BasicHashTableIterator<HashedObject>;

public:
    typedef class BasicHashLink<HashedObject> LinkClass;
    typedef typename LinkClass::Link HashLink;

   /*
    * ====================================================================
    * Implements the I_BasicHashTable interface
    * ====================================================================
    */


   /**
    * @purpose: Construct a BasicHashTable of the given size. The tableSize is
    * often a prime number to allow a better spread of objects across the table
    */
   BasicHashTable(long tableSize);


   /**
    * @purpose: Destruct the BasicHashTable.
    * NOTE: Does not delete the objects, the client owns the objects
    */
   virtual ~BasicHashTable();


   /**
    * @purpose: Return the number of objects stored in the table
    */
   virtual long entries() const { return m_numEntries; }


   /**
    * @purpose: Clear the hash table.
    * NOTE: Does not delete the objects, the client owns the objects
    */
   virtual void clear();


   /**
    * @purpose: Clears the hash table AND DELETEs the objects
    */
   virtual void clearAndDestroy();


   /**
    * @purpose: Finds the given object in the BasicHashTable.
    * Returns 0 if not found
    */
   virtual HashedObject* find(const HashedObject* pItem) const;


   /**
    * @purpose: Inserts the given object into the BasicHashTable.
    * Returns the inserted object.
    * NOTE: No check is made for inserting an object twice
    */
   virtual HashedObject* insert(HashedObject* pItem);


   /**
    * @purpose: Removes the given object from the BasicHashTable.
    * Returns the removed object, or 0 if not found.
    */
   virtual HashedObject* remove(HashedObject* pItem);

#ifdef UTIL_HASH_DEBUG
   virtual void dump() const;
#endif


protected:


    /**
     * @purpose:  Get the bucket of item i.e. the array position
     * in the hash table where it would go
     */
    virtual unsigned long itemBucket(const HashedObject* pItem) const
    {
        return (pItem->hash() % m_tableSize);
    }

    /**
     * @purpose: Get the head Link of the link chain the item would be in
     * NOTE: The chain head link is not used to store an item this saves
     * "if link == head" type checks
     */
    virtual HashLink* chainHead(const HashedObject* pItem) const
    {
        return &m_pTheTable[itemBucket(pItem)];
    }
    /**
     * @purpose:  Clear the BasicHashTable and delete the objects if requested
     */
    virtual void  clearTable(bool deleteObjectsFlag);

protected:
    /**
     * @purpose: The size of BasicHashTable array i.e. this is used to spread
     * objects out with collisions being stored in a linked list. This
     * value is often a prime number to aid the spread of objects.
     */
    unsigned long     m_tableSize;

    /**
     * @purpose: The array used for the BasicHashTable to store the linked
     * lists of objects i.e. the array of linked lists that the objects
     * are hashed into.
     */
    HashLink*         m_pTheTable;

    /**
     * @purpose: The number of objects stored in the BasicHashTable
     */
    unsigned long     m_numEntries;

};


} // namespace

#include "BasicHashTable.I"

#endif


