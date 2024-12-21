#include <iostream>
#include <string>

#include "Link.h"
#include "Linked.h"

//
// Class to test/demo use of Intrusive Linked. Linked is intrusive
// i.e. the objects being linked also hold pointers to
// the Link object. However no extra work is needed i.e.
// the links are automatically created when the link object
// is created and broken on deletion
// 
class Foo : public Util::IntrusiveLinked<Foo> {
public:
    Foo(const std::string& n) : m_name(n) { }
    void tellOther(const std::string& s) {
        if (other()) other()->show(s);
    }
    void show(const std::string& s) { std::cout << m_name << " " << s; }
private:
    const std::string m_name;
};

//
// Class to test/demo linking non-intrusively i.e.
// want to link two Bar objects, but the FooBar is one
// that manages the link between them.
//
class Bar {
public:
    Bar(const std::string& n) : name(n) { }
    std::string name;
};

class FooBar : public Util::NonIntrusiveLinked<Bar> {
public:
    // FooBar owns pBar
    FooBar(const std::string& n) : m_pBar(new Bar(n)) { }
    // Must destroy the link when linked objected destroyed
    virtual ~FooBar() {
        deleteBar();
    }
    void deleteBar() {
        // Must call linkedObjectDead when linked object delete'd
        linkedObjectDead(m_pBar);
        delete m_pBar;
        m_pBar = 0;
    }

    Bar* getBar() { return m_pBar; }
    Bar* getLinkedBar() const { return m_pLink ? m_pLink->other(m_pBar) : 0; }
private:
    Bar* m_pBar;
};

int main()
{
    Foo* pA = new Foo("A");
    Foo* pB = new Foo("B");
    LINKED_TYPE(Foo)::createLink(pA, pB);

    // Call a method on the other end of link
    pA->tellOther("Object A says hello\n");

    // Test the other method
    Foo* pOther = pB->other();
    pOther->show("Talking to A\n");

    // Test the getLink and end1/2 methods
    const LINKED_TYPE(Foo)* pLink = pA->getLink();
    dynamic_cast<Foo*>(pLink->end1())->show("is end1\n");
    dynamic_cast<Foo*>(pLink->end2())->show("is end2\n");

    // Test the link end is removed when object destroyed
    delete pB;
    // - This will not be shown since B has gone
    pA->tellOther("Object A asks if you are alive\n");
    delete pA;
    // NOTE: Link created by createLink delete'd when both A and B deleted


    //
    // Test non-intrusive Link i.e. hand crafted
    //
    FooBar* pFB1 = new FooBar("BAR1");
    FooBar* pFB2 = new FooBar("BAR2");
    Util::Link<Bar>::createLink(pFB1, pFB1->getBar(), pFB2, pFB2->getBar());
    std::cout << "Foobar1's linked BAR = " << pFB1->getLinkedBar()->name << std::endl;
    std::cout << "Foobar2's linked BAR = " << pFB2->getLinkedBar()->name << std::endl;
    pFB2->deleteBar();
    if (pFB1->getLinkedBar())
        std::cout << "EEK Foobar1 still has  linked BAR = " << pFB1->getLinkedBar()->name << std::endl;
    delete pFB1;
    delete pFB2;
    
    return 0;
}
