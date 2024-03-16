#ifndef UTIL_NAMED_OBJECT_H
#define UTIL_NAMED_OBJECT_H

#include <string>

namespace Util {

class NamedObject
{
public:
    NamedObject(const std::string& name) : m_name(name) { }
    virtual ~NamedObject() { }
    const std::string& getName() const { return m_name; }
protected:
    void setName(const std::string& name) { m_name = name; }
private:
    std::string m_name;
};

} // namespace
#endif
