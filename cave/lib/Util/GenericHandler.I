#include <stdlib.h>
#include <string.h>
#include "I_HashTableObject.h"
#include "GenericHandler.h"

namespace Util {

template<typename KT, typename PT, typename RT>
class GenericHandler<KT, PT, RT>::KeyMapping : public Util::I_HashTableObject
{
public:
    KeyMapping(KT key, I_Functor* pF=0) :
        m_key(key), m_pFunctor(0)
    {
        if (pF)
        {
            m_pFunctor = pF->clone();
        }
    }
    virtual ~KeyMapping() { delete m_pFunctor; }

    unsigned long hash() const { return long(m_key); }
    bool isEqual(const I_HashTableObject* pObject) const {
        const KeyMapping* l_pKM = dynamic_cast<const KeyMapping*>(pObject);
        return (l_pKM->m_key == m_key);
    }
    KT m_key;
    I_Functor* m_pFunctor;
};


template<typename KT, typename PT, typename RT>
GenericHandler<KT, PT, RT>::GenericHandler() :
    m_pDefaultHandler(0)
{
    Util::BasicHashLink<KeyMapping>::instance()->setBlockSize(200);
    m_pKeyMap = new Util::BasicHashTable<KeyMapping>(1009);
    memset(&m_noFunctorRetVal, 0x00, sizeof(RT));
}

template<typename KT, typename PT, typename RT>
GenericHandler<KT, PT, RT>::~GenericHandler()
{
    delete m_pDefaultHandler;

    m_pKeyMap->clearAndDestroy();
    delete m_pKeyMap;
}



template<typename KT, typename PT, typename RT>
RT GenericHandler<KT, PT, RT>::execute(KT key, PT param)
{
    I_Functor* l_pF = getFunctorPtr(key);
    if (l_pF)
    {
        return (*l_pF)(param);
    }
    else
    if (m_pDefaultHandler)
    {
        return (*m_pDefaultHandler)(param);
    }
    else
    {
        return m_noFunctorRetVal;
    }
}

template<typename KT, typename PT, typename RT>
typename GenericHandler<KT, PT, RT>::I_Functor* GenericHandler<KT, PT, RT>::getFunctorPtr(KT key)
{
    KeyMapping l_keyMapping(key);
    KeyMapping* l_pFound;

    l_pFound = m_pKeyMap->find(&l_keyMapping);
    return (l_pFound) ? l_pFound->m_pFunctor : 0;
}

template<typename KT, typename PT, typename RT>
void GenericHandler<KT, PT, RT>::registerHandler(KT key, I_Functor* pF)
{
    delete getFunctorPtr(key);
    m_pKeyMap->insert(new KeyMapping(key, pF));
}

template<typename KT, typename PT, typename RT>
void GenericHandler<KT, PT, RT>::setDefaultHandler(I_Functor* pF)
{
    delete m_pDefaultHandler;
    m_pDefaultHandler = (pF) ? pF->clone() : 0;
}

} // namespace

