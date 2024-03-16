#ifndef UTIL_I_ITERATOR_H
#define UTIL_I_ITERATOR_H

namespace Util {

template<typename ITEM>
class I_Iterator
{
protected:
    I_Iterator() { }
public:
    virtual void first() = 0;
    virtual void next() = 0;
    virtual bool isDone() const = 0;
    virtual ITEM currentItem() const = 0;
};

} // namespace
#endif
