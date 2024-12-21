#ifndef UTIL_LINK_H
#define UTIL_LINK_H

#include "NonCopyable.h"

#include "I_SetLink.h"

namespace Util {

//
// VERY basic class to implement a link between two objects
// Each object has a reference to the same Link object and
// updates it when they are destroyed. The Link object delete's
// itself when both objects are deleted.
// Link's must be created via the createLink static method and
// can not be constructed on the stack.
// Clients must not access the link object after calling dead
// (since the Link may have delete'd itself).
//
// Clients MUST implement the I_Linkable interface which provides
// a setLink method (hacky I know). See also the Linked class.
//
template <typename T>
class Link : public NonCopyable<T>
{
private:
    Link() { }
    Link(I_SetLink<T>* pM1, T* p1, I_SetLink<T>* pM2, T* p2) : m_pEnd1(p1), m_pEnd2(p2) {
        // Ensure the objects know they are linked to save the client
        // having to remember to this (hence the I_SetLink interface)
        pM1->setLink(this);
        pM2->setLink(this);
    }
public:
    // Construct an intrusive link i.e. the linked objects also reference the Link object
    // (and must implement the I_SetLink interface
    static Link* createLink(T* p1, T* p2) { return new Link(p1, p1, p2, p2); }
    // Construct a non-intrusive link i.e. pM1 and pM2 implment the I_SetLink and manage
    // the Link object, but the p1,p2 can be any class
    static Link* createLink(I_SetLink<T>* pM1, T* p1, I_SetLink<T>* pM2, T* p2) { return new Link(pM1, p1, pM2, p2); }

    ~Link() { }

    // One end of the link is now dead.
    // If both ends dead then delete this
    void dead(T* p) {
        if (p == m_pEnd1) {
            m_pEnd1 = 0;
            if (not m_pEnd2) delete this;
        }
        else
        if (p == m_pEnd2) {
            m_pEnd2 = 0;
            if (not m_pEnd1) delete this;
        }
    }

    // Get the other end of the link (or 0 if dead)
    T* other(const T* p) const {
        return (p == m_pEnd1) ? m_pEnd2
                : (p == m_pEnd2) ? m_pEnd1 : 0;
    }

    // Just in case needed
    T* end1() const { return m_pEnd1; }
    T* end2() const { return m_pEnd2; }
private:
    T* m_pEnd1;
    T* m_pEnd2;
};

} // namespace

#endif

