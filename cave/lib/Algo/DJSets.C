// Disjoint Set Data Structure
// Author: Emil Stefanov
// Date: 03/28/06
// Implementaton is as described in http://en.wikipedia.org/wiki/Disjoint-set_data_structure

#include "DJSets.h"

#include "DeleteIt.h"
#include "Debug.h"

namespace Algo {

DJSets::DJSets()
{
    Clear();
}

DJSets::DJSets(int count)
{
    Clear();
    AddElements(count);
}

DJSets::DJSets(const DJSets & s)
{
    this->m_numElements = s.m_numElements;
    this->m_numSets = s.m_numSets;

    // Copy nodes
    m_nodes.resize(m_numElements);
    for(int i = 0; i < m_numElements; ++i)
        m_nodes[i] = new Node(*s.m_nodes[i]);

    // Update parent pointers to point to newly created nodes rather than the old ones
    for(int i = 0; i < m_numElements; ++i)
        if(m_nodes[i]->parent)
            m_nodes[i]->parent = m_nodes[s.m_nodes[i]->parent->index];
}

DJSets::~DJSets()
{
    Clear();
}

// Note: some internal data is modified for optimization even though this method is consant.
int DJSets::FindSet(int elementId) const
{
    if ((elementId < 0) or (elementId >= m_numElements)) return -1;

    Node* curNode;

    // Find the root element that represents the set which `elementId` belongs to
    curNode = m_nodes[elementId];
    while(curNode->parent)
        curNode = curNode->parent;
    Node* root = curNode;

    // Walk to the root, updating the parents of `elementId`. Make those elements the direct
    // children of `root`. This optimizes the tree for future FindSet invokations.
    curNode = m_nodes[elementId];
    while(curNode != root)
    {
        Node* next = curNode->parent;
        curNode->parent = root;
        curNode = next;
    }
    //LOG_INFO("SET " << elementId << " = " << root->index);
    return root->index;
}

void DJSets::UnionElementSets(int elementId1, int elementId2)
{
    Union(FindSet(elementId1),FindSet(elementId2));
}

void DJSets::Union(int setId1, int setId2)
{
    if ((setId1 >= m_numElements) || (setId2 >= m_numElements)) return;
    if ((setId1 < 0) || (setId2 < 0)) return;

    if(setId1 == setId2)
        return; // already unioned

    Node* set1 = m_nodes[setId1];
    Node* set2 = m_nodes[setId2];

    // Determine which node representing a set has a higher rank. The node with the higher rank is
    // likely to have a bigger subtree so in order to better balance the tree representing the
    // union, the node with the higher rank is made the parent of the one with the lower rank and
    // not the other way around.
    if(set1->rank > set2->rank)
        set2->parent = set1;
    else if(set1->rank < set2->rank)
        set1->parent = set2;
    else // set1->rank == set2->rank
    {
        set2->parent = set1;
        ++set1->rank; // update rank
    }

    // Since two sets have fused into one, there is now one less set so update the set count.
    --m_numSets;
    //LOG_INFO("UNION " << setId1 << "," << setId2 << " num " << m_numSets);
}

void DJSets::AddElements(int numToAdd)
{
    if (numToAdd < 0) return;

    // insert and initialize the specified number of element nodes to the end of the `m_nodes` array
    m_nodes.insert(m_nodes.end(), numToAdd, (Node*)0);
    for(int i = m_numElements; i < m_numElements + numToAdd; ++i)
    {
        m_nodes[i] = new Node();
        m_nodes[i]->parent = 0;
        m_nodes[i]->index = i;
        m_nodes[i]->rank = 0;
    }

    // update element and set counts
    m_numElements += numToAdd;
    m_numSets += numToAdd;
}

int DJSets::NumElements() const
{
    return m_numElements;
}

int DJSets::NumSets() const
{
    return m_numSets;
}

void DJSets::Clear()
{
    m_numElements = 0;
    m_numSets = 0;
    Util::deleteAndClear(m_nodes);
}

} // namespace

