#ifndef ALGO_DISJOINT_SETS_H
#define ALGO_DISJOINT_SETS_H

#include <map>

//
// This wrapper is used b because DJSets only deals with int's
// i.e. DJSets uses int's for the elements and these are
// grouped into sets identified by int's.
// This maps 'Element' objects to the 'element int's used by DJSets
//

namespace Algo { class DJSets; }

namespace Algo {

template<typename Ele>
class DisjointSets
{
public:
    DisjointSets();
    virtual ~DisjointSets();

    int findSet(const Ele& e) const;

    void joinSets(int setId1, int setId2);

    void addElement(const Ele& e);

    int numElements() const;
    int numSets() const;

    void clear();
private:
    using ElementIdMap = std::map<Ele, int>;
    ElementIdMap m_elementToId;
    DJSets* m_pSets;
};

} // namespace


#include "DisjointSets.I"

#endif

