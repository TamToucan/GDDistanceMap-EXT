#ifndef UTIL_BASIC_STORE_H
#define UTIL_BASIC_STORE_H

#include <map>
#include "BasicStoreIterator.h"

namespace Util {

    //
    // Simple interface for storing a mapping of T1 => T2
    // Used so that could replace with something smarter if needed
    // Never meant to be used externally i.e. only internal implementation
    // of a class will use this. Hence the createIterator is protected
    //
template<typename T1, typename T2>
class BasicStore
{
public:
    BasicStore(T2 notFound) : m_notFoundT2(notFound) { }
    virtual ~BasicStore() { }

    virtual T2 find(const T1& key) const
    {
        typename BasicStoreMap::const_iterator l_itr = m_store.find(key);
        return (l_itr != m_store.end()) ? l_itr->second : m_notFoundT2;
    }

    virtual void add(const T1& key, const T2& value) { m_store[key] = value; }

    virtual T2 remove(const T1& key)
    {
        T2 l_retVal(m_notFoundT2);
        typename BasicStoreMap::iterator l_itr = m_store.find(key);
        if (l_itr != m_store.end())
        {
            l_retVal = l_itr->second;
            m_store.erase(l_itr);
        }
        return l_retVal;
    }
    virtual void clear()
    {
        m_store.clear();
    }

    virtual typename BasicStoreIteratorPtr<T1,T2>::Type getIterator() const
    {
        return typename BasicStoreIteratorPtr<T1,T2>::Type( new BasicStoreIterator<T1,T2>(this) );
    }

protected:
    friend class BasicStoreIterator<T1,T2>;

    using BasicStoreMap = std::map<T1, T2>;
    BasicStoreMap m_store;
    T2 m_notFoundT2;
};


} // namespace

#endif
