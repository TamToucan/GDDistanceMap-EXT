/*
 * NonCopyable.h
 *
 *  Created on: 10-Aug-2010
 *      Author: Tam
 */

#ifndef NONCOPYABLE_H_
#define NONCOPYABLE_H_

namespace Util {

template <typename T>
class NonCopyable
{
protected:
    NonCopyable() { }
    ~NonCopyable() { }
private:
    NonCopyable(const NonCopyable &);
    T& operator=(const T&);
};

} // namespace

#endif /* NONCOPYABLE_H_ */
