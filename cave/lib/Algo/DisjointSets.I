#include <map>
#include <iterator>
#include "DisjointSets.h"
#include "DJSets.h"

namespace Algo {

template <typename Element>
DisjointSets<Element>::DisjointSets()
{
    m_pSets = new DJSets();
}

template<typename Element>
DisjointSets<Element>::~DisjointSets()
{
    delete m_pSets;
}

template<typename Element>
int DisjointSets<Element>::findSet(const Element& e) const
{
    int l_set = -1;
    typename DisjointSets<Element>::ElementIdMap::const_iterator l_itr = m_elementToId.find(e);
    if (l_itr != m_elementToId.end())
    {
        l_set = m_pSets->FindSet((*l_itr).second);
    }
    return l_set;
}

template<typename Element>
void DisjointSets<Element>::joinSets(int setId1, int setId2)
{
    m_pSets->Union(setId1, setId2);
}

template<typename Element>
void DisjointSets<Element>::addElement(const Element& e)
{
    typename DisjointSets<Element>::ElementIdMap::const_iterator l_itr = m_elementToId.find(e);
    if (l_itr == m_elementToId.end())
    {
        m_elementToId[e] = m_pSets->NumElements();
        m_pSets->AddElements(1);
    }
}


template<typename Element>
int DisjointSets<Element>::numElements() const
{
    return m_pSets->NumElements();
}

template<typename Element>
int DisjointSets<Element>::numSets() const
{
    return m_pSets->NumSets();
}

template<typename Element>
void DisjointSets<Element>::clear()
{
    m_elementToId.clear();
    m_pSets->Clear();
}

} // namespace

