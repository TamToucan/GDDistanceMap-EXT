#ifndef UTIL_LINKED_H
#define UTIL_LINKED_H

#include "Link.h"
#include "I_SetLink.h"

namespace Util {

//
// Base class to derive from to add a Link to an object.
// Provides a setLink method for the I_Linkable interface
// and takes care of calling the dead method on destruction
// e.g.
//     class Foo : public Util::IntrusiveLinked<Foo> { .. }
//
//     Foo a;
//     Foo b;
//     // NOTE: Creating a link will call the setLink of a and b
//     LINKED_TYPE::createLink(&a, &b);
//
// It means to access the link (rarely needed) you use
//     const LINKED_TYPE(Foo)* pLink = a.getLink();
//
// The LINKED_TYPE macro is because the link need's to be between
// IntrusiveLinked<Foo> objects NOT 'Foo' This is because in the
// IntrusiveLinked destructor the Foo object will already have been
// destroyed.
//
// The above is an IntrusiveLink, i.e. the linked objects also
// hold the link reference and the link is broken on destruction.
// When another class manages the objects the NonIntrusiveLinked
// is used and the manging class must call linkedObjectDead when
// the linked object is deleted
// e.g.
//
//     class FooBar : public Util::NonIntrusiveLinked<Bar> {
//     ...
//        void deleteBar() {
//            // Must call linkedObjectDead when linked object delete'd
//            linkedObjectDead(m_pBar);
//            delete m_pBar;
//            m_pBar = 0;
//        }
//     private:
//         Bar* m_pBar;
//     };
// 
#define LINKED_TYPE(t) Util::Link< Util::IntrusiveLinked<t> >

template <typename T>
class IntrusiveLinked : public I_SetLink< IntrusiveLinked<T> >
{
public:
    IntrusiveLinked() : m_pLink(0) { }
    virtual ~IntrusiveLinked() { if (m_pLink) m_pLink->dead(this); }

    // I_Linkable interface
    void setLink(LINKED_TYPE(T)* pL) { m_pLink = pL; }

    const LINKED_TYPE(T)* getLink() const { return m_pLink; }

    T* other() const { return (m_pLink) ? dynamic_cast<T*>(m_pLink->other(this)) : 0; }
private:
    LINKED_TYPE(T)* m_pLink;
};


template <typename T>
class NonIntrusiveLinked : public I_SetLink< T >
{
public:
    NonIntrusiveLinked() : m_pLink(0) { }

    // I_Linkable interface
    void setLink(Link< T >* pL) { m_pLink = pL; }

    const Link< T >* getLink() const { return m_pLink; }

    T* other() const { return (m_pLink) ? m_pLink->other(this) : 0; }
protected:
    void linkedObjectDead(T* pObj) { if (m_pLink) m_pLink->dead(pObj); }
protected:
    Util::Link< T >* m_pLink;
};

} // namespace

#endif

