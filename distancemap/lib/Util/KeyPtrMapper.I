#include <vector>
#include <string>
#include <utility>
#include <algorithm>
#include "KeyPtrMapper.h"

#include <map>

namespace Util {

//
// This is the actual class that does the work. Done as a pimpl
// to hide all this rubbish from the header file
//
template<typename KeyType, typename CompFunc>
class KeyPtrMapper<KeyType, CompFunc>::Impl
{
    friend class KeyPtrMapper<KeyType, CompFunc>;

protected:
    //
    // Can't typedef templates => use a struct
    //
    template<typename ItemType>
    struct Type
    {
#if defined(KYM_USE_VECTOR)
        typedef std::pair<KeyType, ItemType*> NameItemPair;
        typedef std::vector< typename Type<ItemType>::NameItemPair > ItemList;
#else
        typedef std::map< KeyType, ItemType*, CompFunc > ItemList;
#endif
    };

    //
    // Template function to add a pointer to the map
    //
    template<typename ItemType>
    void addItem(typename Type<ItemType>::ItemList* pList,
                 KeyType pName, ItemType* pItem)
    {
#if defined(KYM_USE_VECTOR)
        pList->push_back( typename Type<ItemType>::NameItemPair(pName, pItem) );
#else
        (*pList)[pName] = pItem;
#endif
    }



    //
    // Template function to find a pointer in the map
    // NOTE: Could get the template signature to be found if
    // I changed moved ItemType from the parameter list to
    // the return type.
    //
    template<typename ItemType>
    void getItem(typename Type<ItemType>::ItemList* pList,
                 KeyType pName, ItemType** ppItem)
    {
#if defined(KYM_USE_VECTOR)
        for (typename Type<ItemType>::ItemList::iterator l_itr = pList->begin();
                l_itr != pList->end();
                ++l_itr)
            if (l_itr->first == pName) { *ppItem = l_itr->second; return }
        *ppItem = 0;
#else
        typename Type<ItemType>::ItemList::iterator l_itr = pList->find(pName);
        *ppItem = (l_itr != pList->end()) ? l_itr->second : 0;
#endif
    }

protected:
    //
    // The lists (maps) to store the different types of items
    //
    typename Type<int>::ItemList               m_intItems;
    typename Type<unsigned int>::ItemList      m_uintItems;
    typename Type<long>::ItemList              m_longItems;
    typename Type<unsigned long>::ItemList     m_ulongItems;
    typename Type<float>::ItemList             m_floatItems;
    typename Type<bool>::ItemList              m_boolItems;
    typename Type<std::string>::ItemList       m_stringItems;
    typename Type< typename KeyPtrMapper::StringList >::ItemList m_stringListItems;
};

////////////////////////////////////////////////////////////////////////

template<typename KeyType, typename CompFunc>
KeyPtrMapper<KeyType, CompFunc>::KeyPtrMapper()
{
    pimpl = new Impl;
}

template<typename KeyType, typename CompFunc>
KeyPtrMapper<KeyType, CompFunc>::~KeyPtrMapper()
{
    delete pimpl;
}

////////////////////////////////////////////////////////////////////////

//
// Add Methods
//
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addIntPtr(KeyType pName, int* pItem)
{
    pimpl->addItem(&pimpl->m_intItems, pName, pItem);
}
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addUIntPtr(KeyType pName, unsigned int* pItem)
{
    pimpl->addItem(&pimpl->m_uintItems, pName, pItem);
}
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addLongPtr(KeyType pName, long* pItem)
{
    pimpl->addItem(&pimpl->m_longItems, pName, pItem);
}
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addULongPtr(KeyType pName, unsigned long* pItem)
{
    pimpl->addItem(&pimpl->m_ulongItems, pName, pItem);
}
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addFloatPtr(KeyType pName, float* pItem)
{
    pimpl->addItem(&pimpl->m_floatItems, pName, pItem);
}
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addBoolPtr(KeyType pName, bool* pItem)
{
    pimpl->addItem(&pimpl->m_boolItems, pName, pItem);
}
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addStringPtr(KeyType pName, std::string* pItem)
{
    pimpl->addItem(&pimpl->m_stringItems, pName, pItem);
}
template<typename KeyType, typename CompFunc>
void KeyPtrMapper<KeyType, CompFunc>::addStringListPtr(KeyType pName, StringList* pItem)
{
    pimpl->addItem(&pimpl->m_stringListItems, pName, pItem);
}

////////////////////////////////////////////////////////////////////////

//
// Get Methods
// NOTE: Have to use a temp ptr because couldnt get the
// getItem template signature to be found when it returned the pointr
//
template<typename KeyType, typename CompFunc>
int* KeyPtrMapper<KeyType, CompFunc>::getIntPtr(KeyType pName)
{
    int* l_tmpPtr;
    pimpl->getItem(&pimpl->m_intItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}
template<typename KeyType, typename CompFunc>
unsigned int* KeyPtrMapper<KeyType, CompFunc>::getUIntPtr(KeyType pName)
{
    unsigned int* l_tmpPtr;
    pimpl->getItem(&pimpl->m_uintItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}
template<typename KeyType, typename CompFunc>
long* KeyPtrMapper<KeyType, CompFunc>::getLongPtr(KeyType pName)
{
    long* l_tmpPtr;
    pimpl->getItem(&pimpl->m_longItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}
template<typename KeyType, typename CompFunc>
unsigned long* KeyPtrMapper<KeyType, CompFunc>::getULongPtr(KeyType pName)
{
    unsigned long* l_tmpPtr;
    pimpl->getItem(&pimpl->m_ulongItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}
template<typename KeyType, typename CompFunc>
float* KeyPtrMapper<KeyType, CompFunc>::getFloatPtr(KeyType pName)
{
    float* l_tmpPtr;
    pimpl->getItem(&pimpl->m_floatItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}
template<typename KeyType, typename CompFunc>
bool* KeyPtrMapper<KeyType, CompFunc>::getBoolPtr(KeyType pName)
{
    bool* l_tmpPtr;
    pimpl->getItem(&pimpl->m_boolItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}
template<typename KeyType, typename CompFunc>
std::string* KeyPtrMapper<KeyType, CompFunc>::getStringPtr(KeyType pName)
{
    std::string* l_tmpPtr;
    pimpl->getItem(&pimpl->m_stringItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}
template<typename KeyType, typename CompFunc>
typename KeyPtrMapper<KeyType, CompFunc>::StringList*
KeyPtrMapper<KeyType, CompFunc>::getStringListPtr(KeyType pName)
{
    typename KeyPtrMapper<KeyType, CompFunc>::StringList* l_tmpPtr;
    pimpl->getItem(&pimpl->m_stringListItems, pName, &l_tmpPtr);
    return l_tmpPtr;
}

////////////////////////////////////////////////////////////////////////

} // namespace
