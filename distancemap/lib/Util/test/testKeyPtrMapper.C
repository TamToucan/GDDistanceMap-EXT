#include <iostream>
#include <string>
#include "KeyPtrMapper.h"

#include "Containers.h"

int main()
{
    Util::KeyPtrMapper<const char*, Util::ltstr> l_foo;
    int i = 42;
    Util::KeyPtrMapper<const char*, Util::ltstr>::StringList l_strings;
    l_strings.push_back("Hello");
    l_strings.push_back("GoodBye");

    const char* INT1 = "INT1";
    l_foo.addIntPtr(INT1, &i);
    l_foo.addStringListPtr("STRS", &l_strings);

    std::cout << "INT1 = " << *l_foo.getIntPtr(INT1)<< "\n";
    Util::KeyPtrMapper<const char*>::StringList* l_pStrings;
    l_pStrings = l_foo.getStringListPtr("STRS");
    for (Util::KeyPtrMapper<const char*>::StringList::iterator l_itr
                                                       = l_pStrings->begin();
            l_itr != l_pStrings->end();
            ++l_itr)
    {
        std::cout
            << "STR[" << Util::itrToIndex(*l_pStrings,l_itr)
            << "] = " << *l_itr
            << "\n";
    }

    return 0;
}


