#ifndef UTIL_UNIQUE_ID_H
#define UTIL_UNIQUE_ID_H

//
// NOTE: Not thread-safe!
//
namespace Util {

template<typename IDType>
class UniqueID
{
public:
    UniqueID() : m_ID(nextID()) { }
    UniqueID(const UniqueID&) : m_ID(nextID()) { }
    void operator=(const UniqueID&) { }

    const IDType& getUniqueID() const { return m_ID; }
protected:
    IDType m_ID;
private:
    IDType nextID()
    {
        static IDType l_prevID = 0;
        return ++l_prevID;
    }
};

typedef unsigned long ObjectID;
typedef UniqueID<ObjectID> UniqueObjectID;

} // namespace

#endif
