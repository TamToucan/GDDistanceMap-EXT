#ifndef UTIL_I_SET_LINK_H
#define UTIL_I_SET_LINK_H

namespace Util { template <typename T> class Link; }

namespace Util {

//
// This is hacky. To ensure the link's are maintained correctly
// the Link constructor assumes the two objects implement this
// interface to provide a setLink method.
// See also the Linked class which provides further protection.
//
template <typename T>
class I_SetLink
{
public:
    virtual ~I_SetLink() { }
    virtual void setLink(Link<T>* pL) = 0;
};

} // namespace

#endif

