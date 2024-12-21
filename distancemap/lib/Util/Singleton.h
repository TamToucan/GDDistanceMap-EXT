/*
 * Singleton.h
 *
 *  Created on: 10-Aug-2010
 *      Author: Tam
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_

#include "SystemStuff.h"

#include "NonCopyable.h"

namespace Util {

template <typename T>
class Singleton : private Util::NonCopyable<T>
{
public:
    NEVER_INLINE static T* instance()
    {
        static T l_instance;
        return &l_instance;
    }
};

} // namespace

#endif /* SINGLETON_H_ */
