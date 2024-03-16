#ifndef UTIL_KEY_POINTER_MAPPER_H
#define UTIL_KEY_POINTER_MAPPER_H
#include <string>
#include <vector>
#include <functional>
#include <string.h>

//
// This class allows a mapping between a KeyType and a pointer to
// value of type; int, long, float, bool, string or vector of strings.
// It needs a less then function (default == std::less). See STL map docs.
// e.g.
//    int l_myInt = 17;
//    KeyPtrMapper<const char*, Util::ltstr> l_mapper;
//    l_mapper.addIntPtr("MY_INT", &l_myInt);
//    std::cout << "MY INT = " << *l_mapper.getIntPtr("MY_INT");
//
// NOTE:
// It is designed for static mappings which are set up once e.g.
// for config data, since underneath it uses a STL map (which is
// a RB Tree and so re-balances whenever the data changes
//
namespace Util {

    //
    // Define less than function for const char* since useful
    //
    struct ltstr
    {
      bool operator()(const char* s1, const char* s2) const
      {
        return strcmp(s1, s2) < 0;
      }
    };

template<typename KeyType, typename CompFunc=std::less<KeyType> >
class KeyPtrMapper
{
public:
    //
    // A typedef for a list of strings
    //
    typedef std::vector< std::string > StringList;

public:
    KeyPtrMapper();
    ~KeyPtrMapper();

    //
    // The add item methods
    //
    void addIntPtr(KeyType pName, int* pItem);
    void addUIntPtr(KeyType pName, unsigned int* pItem);
    void addLongPtr(KeyType pName, long* pItem);
    void addULongPtr(KeyType pName, unsigned long* pItem);
    void addFloatPtr(KeyType pName, float* pItem);
    void addBoolPtr(KeyType pName, bool* pItem);
    void addStringPtr(KeyType pName, std::string* pItem);
    void addStringListPtr(KeyType pName, StringList* pItem);

    //
    // The get item methods
    //
    int* getIntPtr(KeyType pName);
    unsigned int* getUIntPtr(KeyType pName);
    long* getLongPtr(KeyType pName);
    unsigned long* getULongPtr(KeyType pName);
    float* getFloatPtr(KeyType pName);
    bool* getBoolPtr(KeyType pName);
    std::string* getStringPtr(KeyType pName);
    typename KeyPtrMapper::StringList* getStringListPtr(KeyType pName);

protected:
    class Impl;
    Impl*  pimpl;
};

} // namespace

#include "KeyPtrMapper.I"

#endif
