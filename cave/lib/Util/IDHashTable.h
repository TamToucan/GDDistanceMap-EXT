#ifndef UTIL_ID_HASH_TABLE_H
#define UTIL_ID_HASH_TABLE_H

#include "BasicHashTable.h"
#include "HashedItem.h"

namespace Util {

template<typename IDType>
class IDHashTable : public Util::BasicHashTable< HashedItem<IDType> >
{
public:
    IDHashTable(long tableSize) :
        Util::BasicHashTable< HashedItem<IDType> >(tableSize)
    {
        BasicHashTable< HashedItem<IDType> >::LinkClass::instance()->setBlockSize(1);
    }
    virtual ~IDHashTable() { }

    static IDHashTable<IDType>* instance()
    {
        static IDHashTable<IDType> l_instance(1009);
        return &l_instance;
    }
};

} // namespace

#endif
