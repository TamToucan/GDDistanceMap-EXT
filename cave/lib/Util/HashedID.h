#ifndef UTIL_HASHED_ID_H
#define UTIL_HASHED_ID_H

#include "UniqueID.h"
#include "IDHashTable.h"
#include "HashedItem.h"

#include "Debug.h"

namespace Util {

template<typename IDType>
class HashedID : public Util::UniqueID<IDType>
{
public:
    HashedID()
    {
        LOG_ENTRY_INFO(UniqueID<IDType>::getUniqueID());
        IDHashTable<IDType>::instance()->insert(
                new HashedItem<IDType>(UniqueID<IDType>::getUniqueID()) );
    }
    virtual ~HashedID()
    {
        LOG_ENTRY_INFO(UniqueID<IDType>::getUniqueID());
        HashedItem<IDType> l_hi(UniqueID<IDType>::getUniqueID());
        delete IDHashTable<IDType>::instance()->remove(&l_hi);
    }
    static bool isValidID(IDType id)
    {
        HashedItem<IDType> l_hi(id);
        return IDHashTable<IDType>::instance()->find(&l_hi);
    }
};

typedef HashedID<Util::ObjectID> HashedObjectID;

} // namespace

#endif
