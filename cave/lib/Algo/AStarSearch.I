// used for text debugging
#include <iostream>
#include <cstdio>

#include "AStarSearch.h"
#include "FixedSizeAllocator.h"

namespace Algo {

    // A node represents a possible state in the search
    // The user provided state type is included inside this type
template <class UserState, typename CostType>
class AStarSearch<UserState,CostType>::
    Node
    {
    public:
        Node *parent; // used during the search to record the parent of successor nodes
        Node *child; // used after the search for the application to view the search in reverse

        CostType g; // cost of this node + it's predecessors
        CostType h; // heuristic estimate of distance to goal
        CostType f; // sum of cumulative cost of predecessors and self and heuristic

        Node() :
            parent( 0 ),
            child( 0 ),
            g( 0.0f ),
            h( 0.0f ),
            f( 0.0f )
        {
        }

        UserState m_UserState;
    };


    // For sorting the heap the STL needs compare function that lets us compare
    // the f value of two nodes
template <class UserState, typename CostType>
class AStarSearch<UserState,CostType>::
    HeapCompare_f
    {
    public:
        bool operator() ( const Node *x, const Node *y ) const
        {
            return x->f > y->f;
        }
    };



    // constructor just initialises private data
template <class UserState, typename CostType>
AStarSearch<UserState,CostType>::AStarSearch( int MaxNodes )
    : m_State( NOT_INITIALISED )
    , m_Steps(0)
#ifdef ASTAR_USE_FSA_MEMORY
    , m_FixedSizeAllocator( MaxNodes )
#endif
    , m_CurrentSolutionNode( 0 )
    , m_CancelRequest( false )
#ifdef ASTAR_DEBUG
    , m_AllocateNodeCount(0)
#endif
    {
    }

    // call at any time to cancel the search and free up all the memory
template <class UserState, typename CostType>
void AStarSearch<UserState,CostType>::
    CancelSearch()
    {
        m_CancelRequest = true;
    }

    // Set Start and goal states
template <class UserState, typename CostType>
void AStarSearch<UserState,CostType>::
    SetStartAndGoalStates( UserState &Start, UserState &Goal )
    {
        m_CancelRequest = false;

        m_Start = AllocateNode();
        m_Goal = AllocateNode();

        LOG_ASSERT((m_Start and m_Goal),"Start or Goal 0!");

        m_Start->m_UserState = Start;
        m_Goal->m_UserState = Goal;

        m_State = SEARCHING;

        // Initialise the AStar specific parts of the Start Node
        // The user only needs fill out the state information
        m_Start->g = 0;
        m_Start->h = m_Start->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
        m_Start->f = m_Start->g + m_Start->h;
        m_Start->parent = 0;

        // Push the start node on the Open list
        m_OpenList.push_back( m_Start ); // heap now unsorted

        // Sort back element into heap
        std::push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );

        // Initialise counter for search steps
        m_Steps = 0;
    }

    // Advances search one step
template <class UserState, typename CostType>
typename AStarSearch<UserState, CostType>::State AStarSearch<UserState,CostType>::SearchStep()
    {
        // Firstly break if the user has not initialised the search
        LOG_ASSERT( (m_State > NOT_INITIALISED) and (m_State < INVALID),"Invalid state " << m_State );

        // Next I want it to be safe to do a searchstep once the search has succeeded...
        if ( (m_State == SUCCEEDED) || (m_State == FAILED) )
        {
            return m_State;
        }

        // Failure is defined as emptying the open list as there is nothing left to
        // search or by the user cancelling the search
        if ( m_OpenList.empty() || m_CancelRequest )
        {
            FreeAllNodes();
            m_State = FAILED;
            return m_State;
        }

        // Incremement step count
        ++m_Steps;

        // Pop the best node (the one with the lowest f)
        Node *n = m_OpenList.front(); // get pointer to the node
        std::pop_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
        m_OpenList.pop_back();

        // Check for the goal, once we pop that we're done
        if ( n->m_UserState.IsGoal( m_Goal->m_UserState ) )
        {
            // The user is going to use the Goal Node he passed in
            // so copy the parent pointer of n
            m_Goal->parent = n->parent;

            // A special case is that the goal was passed in as the start state
            // so handle that here
            if ( false == n->m_UserState.IsSameNode( m_Start->m_UserState ) )
            {
                FreeNode( n );

                // set the child pointers in each node (except Goal which has no child)
                Node *nodeChild = m_Goal;
                Node *nodeParent = m_Goal->parent;

                do
                {
                    nodeParent->child = nodeChild;

                    nodeChild = nodeParent;
                    nodeParent = nodeParent->parent;

                }
                while( nodeChild != m_Start ); // Start is always the first node by definition

            }

            // delete nodes that aren't needed for the solution
            FreeUnusedNodes();

            m_State = SUCCEEDED;

            return m_State;
        }
        else // not goal
        {

            // We now need to generate the successors of this node
            // The user helps us to do this, and we keep the new nodes in
            // m_Successors ...

            m_Successors.clear(); // empty vector of successor nodes to n

            // User provides this functions and uses AddSuccessor to add each successor of
            // node 'n' to m_Successors
            bool ret = n->m_UserState.GetSuccessors( this, n->parent ? &n->parent->m_UserState : 0 );

            if ( !ret )
            {

                typename NodeList::iterator successor;

                // free the nodes that may previously have been added
                for( successor = m_Successors.begin(); successor != m_Successors.end(); ++successor )
                {
                    FreeNode( (*successor) );
                }

                m_Successors.clear(); // empty vector of successor nodes to n

                // free up everything else we allocated
                FreeAllNodes();

                m_State = OUT_OF_MEMORY;
                return m_State;
            }

            // Now handle each successor to the current node ...
            for( typename NodeList::iterator successor = m_Successors.begin(); successor != m_Successors.end(); ++successor )
            {

                // 	The g value for this successor ...
                CostType newg = n->g + n->m_UserState.GetCost( (*successor)->m_UserState );

                // Now we need to find whether the node is on the open or closed lists
                // If it is but the node that is already on them is better (lower g)
                // then we can forget about this successor

                // First linear search of open list to find node

                typename NodeList::iterator openlist_result;

                for( openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); ++openlist_result )
                {
                    if ( (*openlist_result)->m_UserState.IsSameNode( (*successor)->m_UserState ) )
                    {
                        break;
                    }
                }

                if ( openlist_result != m_OpenList.end() )
                {
                    // we found this state on open
                    if ( (*openlist_result)->g <= newg )
                    {
                        FreeNode( (*successor) );

                        // the one on Open is cheaper than this one
                        continue;
                    }
                }

                typename NodeList::iterator closedlist_result;

                for( closedlist_result = m_ClosedList.begin(); closedlist_result != m_ClosedList.end(); ++closedlist_result )
                {
                    if ( (*closedlist_result)->m_UserState.IsSameNode( (*successor)->m_UserState ) )
                    {
                        break;
                    }
                }

                if ( closedlist_result != m_ClosedList.end() )
                {
                    // we found this state on closed
                    if ( (*closedlist_result)->g <= newg )
                    {
                        // the one on Closed is cheaper than this one
                        FreeNode( (*successor) );

                        continue;
                    }
                }

                // This node is the best node so far with this particular state
                // so lets keep it and set up its AStar specific data ...

                (*successor)->parent = n;
                (*successor)->g = newg;
                (*successor)->h = (*successor)->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
                (*successor)->f = (*successor)->g + (*successor)->h;

                // Remove successor from closed if it was on it

                if ( closedlist_result != m_ClosedList.end() )
                {
                    // remove it from Closed
                    FreeNode(  (*closedlist_result) );
                    m_ClosedList.erase( closedlist_result );

                    // Fix thanks to ...
                    // Greg Douglas <gregdouglasmail@gmail.com>
                    // who noticed that this code path was incorrect
                    // Here we have found a new state which is already CLOSED
                    // anus

                }

                // Update old version of this node
                if ( openlist_result != m_OpenList.end() )
                {
                    FreeNode( (*openlist_result) );
                       m_OpenList.erase( openlist_result );

                    // re-make the heap
                    // make_heap rather than sort_heap is an essential bug fix
                    // thanks to Mike Ryynanen for pointing this out and then explaining
                    // it in detail. sort_heap called on an invalid heap does not work
                    std::make_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
                }

                // heap now unsorted
                m_OpenList.push_back( (*successor) );

                // sort back element into heap
                std::push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
            }

            // push n onto Closed, as we have expanded it now

            m_ClosedList.push_back( n );

        } // end else (not goal so expand)

         return m_State; // Succeeded bool is false at this point.

    }

    // User calls this to add a successor to a list of successors
    // when expanding the search frontier
template <class UserState, typename CostType>
bool AStarSearch<UserState,CostType>::
    AddSuccessor( UserState &State )
    {
        Node *node = AllocateNode();

        if ( node )
        {
            node->m_UserState = State;

            m_Successors.push_back( node );

            return true;
        }
        return false;
    }

    // Free the solution nodes
    // This is done to clean up all used Node memory when you are done with the
    // search
template <class UserState, typename CostType>
void AStarSearch<UserState,CostType>::
    FreeSolutionNodes()
    {
        Node *n = m_Start;
        if ( m_Start->child )
        {
            do
            {
                Node *del = n;
                n = n->child;
                FreeNode( del );

                del = 0;

            } while( n != m_Goal );

            FreeNode( n ); // Delete the goal
        }
        else
        {
            // if the start node is the solution we need to just delete the start and goal
            // nodes
            FreeNode( m_Start );
            FreeNode( m_Goal );
        }
    }

    // Functions for traversing the solution

    // Get start node
template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetSolutionStart()
    {
        m_CurrentSolutionNode = m_Start;
        return ( m_Start ) ?  &m_Start->m_UserState : 0;
    }

    // Get next node
template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetSolutionNext()
    {
        if ( m_CurrentSolutionNode )
        {
            if ( m_CurrentSolutionNode->child )
            {
                Node *child = m_CurrentSolutionNode->child;

                m_CurrentSolutionNode = m_CurrentSolutionNode->child;

                return &child->m_UserState;
            }
        }
        return 0;
    }

    // Get end node
template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetSolutionEnd()
    {
        m_CurrentSolutionNode = m_Goal;
        return ( m_Goal ) ? &m_Goal->m_UserState : 0;
    }

    // Step solution iterator backwards
template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetSolutionPrev()
    {
        if ( m_CurrentSolutionNode )
        {
            if ( m_CurrentSolutionNode->parent )
            {

                Node *parent = m_CurrentSolutionNode->parent;

                m_CurrentSolutionNode = m_CurrentSolutionNode->parent;

                return &parent->m_UserState;
            }
        }
        return 0;
    }

    // Get the number of steps
template <class UserState, typename CostType>
int AStarSearch<UserState,CostType>::
    GetStepCount()
    {
        return m_Steps;
    }


#ifdef ASTAR_DEBUG
    // For educational use and debugging it is useful to be able to view
    // the open and closed list at each step, here are two functions to allow that.
template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetOpenListStart()
    {
        CostType f,g,h;
        return GetOpenListStart( f,g,h );
    }

template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetOpenListStart( CostType& f, CostType& g, CostType& h )
    {
        iterDbgOpen = m_OpenList.begin();
        if ( iterDbgOpen != m_OpenList.end() )
        {
            f = (*iterDbgOpen)->f;
            g = (*iterDbgOpen)->g;
            h = (*iterDbgOpen)->h;
            return &(*iterDbgOpen)->m_UserState;
        }
        return 0;
    }

template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetOpenListNext()
    {
        CostType f,g,h;
        return GetOpenListNext( f,g,h );
    }

template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetOpenListNext( CostType& f, CostType& g, CostType& h )
    {
        ++iterDbgOpen;
        if ( iterDbgOpen != m_OpenList.end() )
        {
            f = (*iterDbgOpen)->f;
            g = (*iterDbgOpen)->g;
            h = (*iterDbgOpen)->h;
            return &(*iterDbgOpen)->m_UserState;
        }
        return 0;
    }

template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetClosedListStart()
    {
        CostType f,g,h;
        return GetClosedListStart( f,g,h );
    }

template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetClosedListStart( CostType& f, CostType& g, CostType& h )
    {
        iterDbgClosed = m_ClosedList.begin();
        if ( iterDbgClosed != m_ClosedList.end() )
        {
            f = (*iterDbgClosed)->f;
            g = (*iterDbgClosed)->g;
            h = (*iterDbgClosed)->h;

            return &(*iterDbgClosed)->m_UserState;
        }
        return 0;
    }

template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetClosedListNext()
    {
        CostType f,g,h;
        return GetClosedListNext( f,g,h );
    }

template <class UserState, typename CostType>
UserState* AStarSearch<UserState,CostType>::
    GetClosedListNext( CostType& f, CostType& g, CostType& h )
    {
        ++iterDbgClosed;
        if ( iterDbgClosed != m_ClosedList.end() )
        {
            f = (*iterDbgClosed)->f;
            g = (*iterDbgClosed)->g;
            h = (*iterDbgClosed)->h;

            return &(*iterDbgClosed)->m_UserState;
        }
        return 0;
    }

template <class UserState, typename CostType>
void AStarSearch<UserState,CostType>::
    EnsureMemoryFreed()
    {
#ifdef ASTAR_USE_FSA_MEMORY
        LOG_ASSERT(m_AllocateNodeCount == 0, "All memory NOT freed");
#endif
    }
#endif


////////////////////////////////////////////////////////////////////////////
// Private Methods
////////////////////////////////////////////////////////////////////////////

    // This is called when a search fails or is cancelled to free all used
    // memory
template <class UserState, typename CostType>
void AStarSearch<UserState,CostType>::FreeAllNodes()
    {
        // iterate open list and delete all nodes
        typename NodeList::iterator iterOpen = m_OpenList.begin();

        while( iterOpen != m_OpenList.end() )
        {
            Node *n = (*iterOpen);
            FreeNode( n );

            ++iterOpen;
        }

        m_OpenList.clear();

        // iterate closed list and delete unused nodes
        typename NodeList::iterator iterClosed;

        for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); ++iterClosed )
        {
            Node *n = (*iterClosed);
            FreeNode( n );
        }

        m_ClosedList.clear();

        // delete the goal

        FreeNode(m_Goal);
    }

////////////////////////////////////////////////////////////////////////////

template <class UserState, typename CostType>
void AStarSearch<UserState,CostType>::FreeUnusedNodes()
    {
        // iterate open list and delete unused nodes
        typename NodeList::iterator iterOpen = m_OpenList.begin();

        while( iterOpen != m_OpenList.end() )
        {
            Node *n = (*iterOpen);

            if ( !n->child )
            {
                FreeNode( n );

                n = 0;
            }

            ++iterOpen;
        }

        m_OpenList.clear();

        // iterate closed list and delete unused nodes
        typename NodeList::iterator iterClosed;

        for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); ++iterClosed )
        {
            Node *n = (*iterClosed);

            if ( !n->child )
            {
                FreeNode( n );
                n = 0;
            }
        }

        m_ClosedList.clear();

    }

////////////////////////////////////////////////////////////////////////////

    // Node memory management
template <class UserState, typename CostType>
typename AStarSearch<UserState,CostType>::Node* AStarSearch<UserState,CostType>::AllocateNode()
    {

#ifndef ASTAR_USE_FSA_MEMORY
        Node *p = new Node;
        return p;
#else
        Node *address = m_FixedSizeAllocator.alloc();

        if ( !address )
        {
            return 0;
        }
#ifdef ASTAR_DEBUG
        ++m_AllocateNodeCount;
#endif
        Node *p = new (address) Node;
        return p;
#endif
    }

////////////////////////////////////////////////////////////////////////////

template <class UserState, typename CostType>
void AStarSearch<UserState,CostType>::FreeNode( AStarSearch<UserState,CostType>::Node* node )
    {
#ifdef ASTAR_DEBUG
        --m_AllocateNodeCount
#endif
#
#ifndef ASTAR_USE_FSA_MEMORY
        delete node;
#else
        m_FixedSizeAllocator.free( node );
#endif
    }

////////////////////////////////////////////////////////////////////////////

} // namespace


