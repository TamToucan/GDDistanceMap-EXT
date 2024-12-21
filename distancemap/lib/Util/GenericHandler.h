#ifndef UTIL_GENERIC_HANDLER_H
#define UTIL_GENERIC_HANDLER_H

#include <functional>
#include "BasicHashTable.h"

//
// I think I hate this class, but I'm not sure...
//
// What it does is allow a mapping to be defined between
// a "KeyType" and an object derived from "I_Functor".
// When the "execute" method is called with a key the
// appropriate I_Functor object is found and it's function
// method ( operator() ) called passing in the ParamType
// object passed to execute and returning RetType.
//
// The use of I_Functor allows greater flexibility than
// just mapping to (say) a static function.
//
// It uses the BasicHashTable for the key to functor mapping
// so the KeyType has to have an operator== and has to be
// convertible to a long so it can be used as a hash. This
// is particularly crap, but since I only need to map from
// int's I left it as is.
//
// Also, memset (0) is called on a "RetType" (m_noFunctorRetVal)
// to use as the return value if execute is called using a key
// which has no functor associated with it (and no default handler
// has been set).
//
// At first I thought it should be an error, but it depends what
// this is being used for e.g. I now use it for the FSM and it's
// valid to get an event which is ignored by the FSM.
//
// The I_Functor class must implement the operator() method
// and a clone method. The clone is because this class copies
// the functor when the association is set up using registerHandler.
//

namespace Util {

template<typename KeyType, typename ParamType, typename RetType>
class GenericHandler
{
protected:
    // Value returned by execute if no functor found
    RetType m_noFunctorRetVal;

public:
    // The I_Functor interface has an operator() method
    // and a clone method (so this class can own the functor)
    struct I_Functor : public std::unary_function<ParamType, RetType>
    {
        virtual RetType operator()(ParamType p) = 0;
        virtual I_Functor* clone() const = 0;
    };

    GenericHandler();

    virtual ~GenericHandler();

    // Find the functor associated with the key
    // and call it's operator() method passing in
    // the param and returning it's return value
    // If no functor found a 0'ed RetType is returned
    //
    RetType execute(KeyType key, ParamType param);

    // Get the functor for the given key
    I_Functor* getFunctorPtr(KeyType key);

    // Associate the functor as the handler for the key
    // NOTE: A copy is make of the functor so the caller
    // need not keep the object
    void registerHandler(KeyType key, I_Functor* pFunctor);

    // Associate the functor as the default handler
    // This is called if no key/functor association is found.
    // Can set to 0 to have no default.
    // NOTE: A copy is make of the functor so the caller
    // need not keep the object
    void setDefaultHandler(I_Functor* pFunctor);

protected:
    // HashTable to store the association of keys to functors
    class KeyMapping;
    Util::BasicHashTable<KeyMapping>*  m_pKeyMap;

    // Default handler to call if no key/functor mapping found
    I_Functor* m_pDefaultHandler;
};

} // namespace

#include "GenericHandler.I"

#endif

