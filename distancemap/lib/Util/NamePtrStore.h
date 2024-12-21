#ifndef UTIL_NAME_PTR_STORE_H
#define UTIL_NAME_PTR_STORE_H

#include <string>

#include "BasicStore.h"
#include "NamePtrStoreIterator.h"

#include "Debug.h"

namespace Util {

    //template <typename OBJTYPE> class NamePtrStoreIterator;

    //
    // This just stores OBJTYPE ptrs using a name. Provides the simple interface
    //     exits
    //     add
    //     get
    //     destroy
    //
    // It owns the objects added and deletes them when removed or cleared
    // Uses BasicStore as the storage
    //
template<typename OBJTYPE>
class NamePtrStore : public Util::BasicStore<std::string, OBJTYPE*>
{
public:
    // Debug Name is used to prefixed debug output
    //
    NamePtrStore(const std::string& debugName, OBJTYPE* pNotFound=0);

    virtual ~NamePtrStore();

    // Check if object using name exists already
    //
    virtual bool exists(const std::string& name) const;

    // Retrieve object using name, 0 if not found
    //
    virtual OBJTYPE* get(const std::string& name) const;

    // NOTE: Owns object after being added.
    // NOTE: Doesn't overwrite any existing Object. Check if returned
    // Object is different from input to see (or call exists before this)
    //
    virtual OBJTYPE* add(const std::string& name, OBJTYPE* pObj);

    // Remove and Delete existing Object (Need to be sure no one has reference)
    //
    virtual bool destroy(const std::string& name);

    // Delete all the objects
    //
    virtual void clear();

    // Get an iterator (for the the Names and objects)
    //
    virtual typename NamePtrStoreIteratorPtr<OBJTYPE>::Type getPairIterator() const;

    // Inherited iterator (returns objects)
    //
    // virtual typename BasicStoreIteratorPtr<std::string, OBJTYPE>::Type getIterator() const;

protected:
    friend class NamePtrStoreIterator<OBJTYPE>;

    const std::string   m_debugName;
};

} // namespace

#include "NamePtrStore.I"

#endif
