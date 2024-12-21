#include <string>

#include "NamePtrStore.h"
#include "BasicStore.h"
#include "BasicStoreIterator.h"
#include "NamePtrStoreIterator.h"

#include "Debug.h"

namespace Util {

template<typename OBJTYPE>
NamePtrStore<OBJTYPE>::NamePtrStore(const std::string& debugName, OBJTYPE* pNotFound) :
    Util::BasicStore<std::string, OBJTYPE*>(pNotFound),
    m_debugName(debugName)
{
    LOG_INFO(m_debugName << " - CREATED");
}

template<typename OBJTYPE>
NamePtrStore<OBJTYPE>::~NamePtrStore()
{
    LOG_INFO(m_debugName << " - DESTROYED");
    clear();
}

///////////////////////////////////////////////////////////////////////

template<typename OBJTYPE>
bool NamePtrStore<OBJTYPE>::exists(const std::string& name) const
{
    return (BasicStore<std::string, OBJTYPE*>::m_notFoundT2 != BasicStore<std::string, OBJTYPE*>::find(name));
}

///////////////////////////////////////////////////////////////////////

template<typename OBJTYPE>
OBJTYPE* NamePtrStore<OBJTYPE>::get(const std::string& name) const
{
    return BasicStore<std::string, OBJTYPE*>::find(name);
}

///////////////////////////////////////////////////////////////////////

template<typename OBJTYPE>
OBJTYPE* NamePtrStore<OBJTYPE>::add(const std::string& name, OBJTYPE* pObj)
{
    OBJTYPE* l_pObj = get(name);
    if (BasicStore<std::string, OBJTYPE*>::m_notFoundT2 == l_pObj)
    {
        if (BasicStore<std::string, OBJTYPE*>::m_notFoundT2 != pObj)
        {
            BasicStore<std::string, OBJTYPE*>::add(name, pObj);
            LOG_INFO(m_debugName << "::add - ADDED " << name);
        }
        l_pObj = pObj;
    }
    return l_pObj;
}

///////////////////////////////////////////////////////////////////////

template<typename OBJTYPE>
bool NamePtrStore<OBJTYPE>::destroy(const std::string& name)
{
    bool l_retVal = false;
    OBJTYPE* l_pObj = BasicStore<std::string, OBJTYPE*>::remove(name);
    if  (l_pObj)
    {
        LOG_INFO(m_debugName << "::destroy - DESTROYED " << name);
        delete l_pObj;
        l_retVal = true;
    }
    return l_retVal;
}

///////////////////////////////////////////////////////////////////////

template<typename OBJTYPE>
void NamePtrStore<OBJTYPE>::clear()
{
    LOG_INFO(m_debugName << "::clear");
    basicStoreDeleteAndClear(*this);
}

///////////////////////////////////////////////////////////////////////

template<typename OBJTYPE>
typename NamePtrStoreIteratorPtr<OBJTYPE>::Type NamePtrStore<OBJTYPE>::getPairIterator() const
{
    return typename NamePtrStoreIteratorPtr<OBJTYPE>::Type( new NamePtrStoreIterator<OBJTYPE>(this) );
}

///////////////////////////////////////////////////////////////////////

} // namespace
