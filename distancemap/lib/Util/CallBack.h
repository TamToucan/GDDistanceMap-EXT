#ifndef UTIL_CALLBACK_H
#define UTIL_CALLBACK_H

namespace Util {

    //
    // Function object to provide a callback method on a Xarg member function
    //
    // O = Object type to call member function of
    // R = Return type of the member function
    // A1 = Arg1 type of the member function
    // A2 = Arg2 type of the member function
    // Ax = ArgX type of the member function
    //
    // make_CallBackX helper functions are provided
    //
    // Example of use
#if 0
class Foo {
public:
    int memFunc1(int i, const char* pStr);
    int memFunc2(int i, const char* pStr);
};

void test()
{
    Foo a;
    // CallBack to a Foo member function returning an Int and taking 2 args
    // an int and a const char*
    typedef Util::CallBack2<Foo, int, int, const char*> Int_ARGs_IntConstCharStar_CB;

    Int_ARGs_IntConstCharStar_CB l_CBObject1(&a, &Test::memFunc1);
    Int_ARGs_IntConstCharStar_CB l_CBObject2;
    l_CBObject2 = make_CallBack2(&a, &Test::memFunc2);

    // Call memFunc1 on object 'a' passing in 9 and "Test 1"
    l_CBObject1.callback(9, "Test 1");
    // Call memFunc2 on object 'a' passing in 11 and "Test 2"
    l_CBObject2.callback(11, "Test 2");

    return 0;
}
#endif



//
// ZERO ARGUEMENTS
//
template< class O, typename R >
class CallBack0
{
public:
    CallBack0() : m_pObj( 0 ), m_pfCB( 0 ) { }
    CallBack0( O* pObj, R (O::*pfCB)(void) ) : m_pObj( pObj ), m_pfCB( pfCB ) { }

    R callback( void ) { return (m_pObj->*m_pfCB)(); }
private:
    O*  m_pObj;
    R (O::*m_pfCB)(void);
};

// Maker helper for CallBack0
//
template< class O, typename R >
CallBack0< O, R >
make_CallBack0(O* pObj, R (O::*pfCB)(void) )
{
    return CallBack0<O, R>(pObj, pfCB);
}

//
// SINGLE ARGUEMENT
//
template< class O, typename R, typename A1 >
class CallBack1
{
public:
    CallBack1() : m_pObj( 0 ), m_pfCB( 0 ) { }
    CallBack1( O* pObj, R (O::*pfCB)(A1) ) : m_pObj( pObj ), m_pfCB( pfCB ) { }

    R callback( A1 arg1 ) { return (m_pObj->*m_pfCB)(arg1); }
private:
    O*  m_pObj;
    R (O::*m_pfCB)(A1);
};

// Maker helper for CallBack1
//
template< class O, typename R, typename A1 >
CallBack1< O, R, A1 >
make_CallBack1(O* pObj, R (O::*pfCB)(A1) )
{
    return CallBack1<O, R, A1>(pObj, pfCB);
}

//
// TWO ARGUEMENTS
//
template< class O, typename R, typename A1, typename A2 >
class CallBack2
{
public:
    CallBack2() : m_pObj( 0 ), m_pfCB( 0 ) { }
    CallBack2( O* pObj, R (O::*pfCB)(A1,A2) ) : m_pObj( pObj ), m_pfCB( pfCB ) { }

    R callback( A1 arg1, A2 arg2 ) { return (m_pObj->*m_pfCB)(arg1,arg2); }
private:
    O*  m_pObj;
    R (O::*m_pfCB)(A1,A2);
};

// Maker helper for CallBack2
//
template< class O, typename R, typename A1, typename A2 >
CallBack2< O, R, A1, A2 >
make_CallBack2(O* pObj, R (O::*pfCB)(A1,A2) )
{
    return CallBack2<O, R, A1, A2>(pObj, pfCB);
}

} // namespace

#endif
