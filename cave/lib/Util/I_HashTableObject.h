#ifndef UTIL_I_HASH_TABLE_OBJECT
#define UTIL_I_HASH_TABLE_OBJECT

namespace Util {

class I_HashTableObject {
public:
    virtual unsigned long hash() const = 0;
    virtual bool isEqual(const I_HashTableObject* pObject) const = 0;
};

} // namespace

#endif
