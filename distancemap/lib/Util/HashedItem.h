#ifndef UTIL_HASHED_ITEM_H
#define UTIL_HASHED_ITEM_H

#include "I_HashTableObject.h"

namespace Util {

template<typename ItemType>
class HashedItem : public Util::I_HashTableObject
{
    ItemType m_item;
public:
    HashedItem(ItemType item) : m_item(item) { }
    virtual ~HashedItem() { }
    virtual unsigned long hash() const { return static_cast<unsigned long>(m_item); }
    virtual bool isEqual(const I_HashTableObject* pObject) const
    {
        const HashedItem* l_pRHS = dynamic_cast<const HashedItem*>(pObject);
        return (m_item == l_pRHS->m_item);
    }
};

} // namespace

#endif

