/*
A* Algorithm Implementation using STL is
Copyright (C)2001-2005 Justin Heyes-Jones

Permission is given by the author to freely redistribute and
include this code in any program as long as this credit is
given where due.

  COVERED CODE IS PROVIDED UNDER THIS LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
  INCLUDING, WITHOUT LIMITATION, WARRANTIES THAT THE COVERED CODE
  IS FREE OF DEFECTS, MERCHANTABLE, FIT FOR A PARTICULAR PURPOSE
  OR NON-INFRINGING. THE ENTIRE RISK AS TO THE QUALITY AND
  PERFORMANCE OF THE COVERED CODE IS WITH YOU. SHOULD ANY COVERED
  CODE PROVE DEFECTIVE IN ANY RESPECT, YOU (NOT THE INITIAL
  DEVELOPER OR ANY OTHER CONTRIBUTOR) ASSUME THE COST OF ANY
  NECESSARY SERVICING, REPAIR OR CORRECTION. THIS DISCLAIMER OF
  WARRANTY CONSTITUTES AN ESSENTIAL PART OF THIS LICENSE. NO USE
  OF ANY COVERED CODE IS AUTHORIZED HEREUNDER EXCEPT UNDER
  THIS DISCLAIMER.

  Use at your own risk!

*/
#ifndef ASTAR_SEARCH_H
#define ASTAR_SEARCH_H

#include <algorithm>
#include <set>
#include <vector>

#include "Debug.h"

// Fixed size memory allocator can be disabled to compare performance
// Uses std new and delete instead if you turn it off
//#define ASTAR_USE_FSA_MEMORY
//#define ASTAR_DEBUG

#ifdef ASTAR_USE_FSA_MEMORY
// fast fixed size memory allocator, used for fast node memory management
#include "FixedSizeAllocator.h"
#endif

//
// To use this algorithm a 'UserState' class must be implemented by the client
// to represent each node in the search space. It must have the following interface
//
/*
    // Cost of getting from this node to the goalNode
    CostType GoalDistanceEstimate( UserState& goalNode );

    // Is this node the goalNode
    bool IsGoal( UserState& goalNode );

    // Add the successor nodes
    // For each successor to this node call the AStarSearch's AddSuccessor to
    // add them the current search
    // pParent is the parent of this node (maybe 0)
    // Returns FALSE to indicate OUT_OF_MEMORY
    bool GetSuccessors( AStarSearch<UserState, CostType>* pAStarSearch, UserState* pParent);

    // Get the cost (g) of going from this node to the successor node
    CostType GetCost( UserState& successor );

    // Is this node the same as rhs
    bool IsSameNode( UserState& rhs );
*/

namespace Algo {

//
// The AStar search class
// UserState is the users state space type
// CostType is how movement cost's are measured  e.g. int, float
//
template <class UserState, typename CostType>
class AStarSearch
{
public:
    enum State
    {
        NOT_INITIALISED,
        SEARCHING,
        SUCCEEDED,
        FAILED,
        OUT_OF_MEMORY,
        INVALID
    };
public:
    AStarSearch( int MaxNodes = 1000 );

    // call at any time to cancel the search and free up all the memory
    void CancelSearch();

    // Set Start and goal states
    void SetStartAndGoalStates( UserState& Start, UserState& Goal );

    // Advances search one step
    State SearchStep();

    // User calls this to add a successor to a list of successors
    // when expanding the search frontier
    // Returns FALSE to indicate OUT_OF_MEMORY
    //
    bool AddSuccessor( UserState& State );

    // Free the solution nodes
    // Clean up all used Node memory when you are done with the search
    void FreeSolutionNodes();

    //
    // Functions for traversing the solution
    //
    UserState* GetSolutionStart();
    UserState* GetSolutionNext();
    UserState* GetSolutionEnd();
    UserState* GetSolutionPrev();

    int GetStepCount();

    // For educational use and debugging it is useful to be able to view
    // the open and closed list at each step, here are two functions to allow that.
#ifdef ASTAR_DEBUG
    UserState* GetOpenListStart();
    UserState* GetOpenListStart( CostType& f, CostType& g, CostType& h );

    UserState* GetOpenListNext();
    UserState* GetOpenListNext( CostType& f, CostType& g, CostType& h );

    UserState* GetClosedListStart();
    UserState* GetClosedListStart( CostType& f, CostType& g, CostType& h );

    UserState* GetClosedListNext();
    UserState* GetClosedListNext( CostType& f, CostType& g, CostType& h );

    void ensureMemoryFreed();
#endif

private:
    class Node;
    class HeapCompare_f;

    // This is called when a search fails or is cancelled to free all used
    // memory
    void FreeAllNodes();

    // This call is made by the search class when the search ends. A lot of nodes may be
    // created that are still present when the search ends. They will be deleted by this
    // routine once the search ends
    void FreeUnusedNodes();

    // Node memory management
    Node *AllocateNode();

    void FreeNode( Node *node );

private:
    typedef std::vector< Node* > NodeList;

    // Heap (simple vector but used as a heap, cf. Steve Rabin's game gems article)
    NodeList m_OpenList;

    // Closed list is a vector.
    NodeList m_ClosedList;

    // Successors is a vector filled out by the user each type successors to a node
    // are generated
    NodeList m_Successors;

    // State
    State m_State;

    // Counts steps
    int m_Steps;

    Node *m_CurrentSolutionNode;

    bool m_CancelRequest;

    // Start and goal state pointers
    Node *m_Start;
    Node *m_Goal;

#ifdef  ASTAR_USE_FSA_MEMORY
    // Memory
     FixedSizeAllocator<Node> m_FixedSizeAllocator;
#endif

#ifdef ASTAR_DEBUG
    typename NodeList::iterator iterDbgOpen;
    typename NodeList::iterator iterDbgClosed;

    // count memory allocation and free's
    int m_AllocateNodeCount;
#endif
};

} // namespace

#include "AStarSearch.I"

#endif

