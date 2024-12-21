#ifndef UTIL_I_HASH_TABLE_H
#define UTIL_I_HASH_TABLE_H

// --------------------------------------------------------------------------
// Filename    : I_HashTable.h
// Project     : Util
// Description : Hash Table Interface Definition
//
// NOTE: This HashTable stores POINTERS to the objects not the objects.
// Therefore you would use generally use
//
//    I_HashTable<MyClass> hashTable
//
// and NOT
//
//    I_HashTable<MyClass*> hashTable
// --------------------------------------------------------------------------

namespace Util {

template< class HashedObject >
class I_HashTable
{
public:
   /**
    * @purpose: Destruct the HashTable.
    * NOTE: Does not delete the objects, the client owns the objects
    */
   virtual ~I_HashTable() { };


   /**
    * @purpose: Return the number of objects stored in the table
    */
   virtual long entries() const = 0;


   /**
    * @purpose: Clear the hash table.
    * NOTE: Does not delete the objects, the client owns the objects
    */
   virtual void clear() = 0;


   /**
    * @purpose: Clears the hash table AND DELETEs the objects
    */
   virtual void clearAndDestroy() = 0;


   /**
    * @purpose: Finds the given object in the HashTable.
    * Returns 0 if not found
    */
   virtual HashedObject* find(const HashedObject* pItem) const = 0;


   /**
    * @purpose: Inserts the given object into the HashTable.
    * Returns the inserted object.
    * NOTE: No check is made for inserting an object twice
    */
   virtual HashedObject* insert(HashedObject* pItem) = 0;


   /**
    * @purpose: Removes the given object from the HashTable.
    * Returns the removed object, or 0 if not found.
    */
   virtual HashedObject* remove(HashedObject* pItem) = 0;
};

} // namespace

#endif


