#include <string>
#include <iterator>
#include <algorithm>
#include <iostream>
#include <vector>

#include "TwoWayMapping.h"

using namespace Util;

//
// Functor to print stuff e.g. use with for_each
//
    template<class T> struct print : public std::unary_function<T, void>
    {
        print(std::ostream& os, const char* pre="", const char* pst="\n") :
            m_os(os), m_pre(pre), m_pst(pst) {}
        void operator() (T x) { m_os << m_pre << x << m_pst; }
        std::ostream& m_os;
        const char* m_pre;
        const char* m_pst;
    };

//
// ostream operator for pairs
//
    template<typename T1, typename T2>
    std::ostream& operator<<(std::ostream& rOS, const std::pair<T1, T2>& e)
    {
        return rOS << e.first << " " << e.second;
    }


int main()
{
    typedef TwoWayMapping<char, std::string> CharStringMapping;
    CharStringMapping c2s('\0',"");

    c2s.add(' ',"space");
    c2s.add('1',"one");

    std::cout << c2s.convert(' ') << "\n";
    std::cout << c2s.convert("soace") << "\n";
    std::cout << c2s.convert('1') << "\n";
    std::cout << c2s.convert("one") << "\n";
    std::cout << c2s.convert('0') << "\n";
    if (c2s.exists('1'))
    {
        std::cout << " ONE EXIST\n";
    }
    if (c2s.exists('0'))
    {
        std::cout << " 0 EXIST\n";
    }
    if (c2s.exists("space"))
    {
        std::cout << " SPACE EXIST\n";
    }
    if (c2s.exists("Z"))
    {
        std::cout << " Z EXIST\n";
    }

    // Iterate over T1=>T2 mapping
    {
        std::cout << "### T1=>T2 mappings\n";
        CharStringMapping::const_T1toT2_iterator itr = c2s.T1toT2_begin();
        while (itr != c2s.T1toT2_end())
        {
            const CharStringMapping::T1toT2_value_type& rV = *itr;
            std::cout << "'" << rV.first << "' => " << rV.second << "\n";
            ++itr;
        }
        std::cout << "### T1=>T2 mappings\n";
        // Or just use
        std::for_each(c2s.T1toT2_begin(), c2s.T1toT2_end(),
                      print<CharStringMapping::T1toT2_value_type>(std::cout));
    }
    // As above
    std::cout << "### T2=>T1 mappings\n";
    std::for_each(c2s.T2toT1_begin(), c2s.T2toT1_end(),
            print<CharStringMapping::T2toT1_value_type>(std::cout));

    std::cout << "### T1s\n";
    std::for_each(c2s.T1_begin(), c2s.T1_end(), print<CharStringMapping::T1_value_type>(std::cout,"'","'\n"));

    std::cout << "### T2s\n";
    std::for_each(c2s.T2_begin(), c2s.T2_end(), print<CharStringMapping::T2_value_type>(std::cout));


    //
    // Example of use iterator_to_member
    //
    {
        typedef std::vector< std::pair<char, std::string> > VCSPair;
        typedef iterator_to_member< VCSPair::iterator, VCSPair::value_type, char > I2M;

        VCSPair v;
        v.push_back(std::make_pair('1',"one"));
        v.push_back(std::make_pair('2',"two"));
        v.push_back(std::make_pair('3',"three"));
        I2M itr = make_iterator_to_member( v.begin(), &std::pair<char, std::string>::first );

        std::cout << "\n\nI2M 2nd entry T1 = '" << *(itr + 1) << "'\n";
        itr += 2;
        std::cout << "I2M 3rd entry T1 = '" << *itr << "'\n";

        std::for_each(make_iterator_to_member( v.begin(), &std::pair<char, std::string>::second),
                      make_iterator_to_member( v.end(), &std::pair<char, std::string>::second),
                      print<std::string>(std::cout));
    }
    return 0;
}
