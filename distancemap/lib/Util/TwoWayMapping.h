
#ifndef UTIL_TWO_WAY_MAPPING_H
#define UTIL_TWO_WAY_MAPPING_H

#include <map>
#include <iterator>

#include "iterator_to_member.h"

namespace Util {

    //
    // Define a two-way mapping of T1's and T2's
    //
    // NOTE: T1 and T2 must be of DIFFERENT types
    // Lots of iterators are provided
    // - Can iterate over the T1 => T2 mappings getting pair<T1,T2>'s
    // - Can iterate over the T2 => T1 mappings getting pair<T2,T1>'s
    // - Can iterate over just the T1's
    // - Can iterate over just the T2's
    //
    // For example, print all the strings in a char/string mapping
    //
    //    typedef TwoWayMapping<char, std::string> CharStringMapping;
    //    CharStringMapping c2s('\0',"");
    //    c2s.add('0',"zero");
    //    c2s.add('1',"one");
    //
    //    CharStringMapping::const_T2_iterator itr = c2s.T2_begin();
    //    while (itr != c2s.T2_end())
    //    {
    //        // Could just use const string&, but T2_value_type means if
    //        // different mapping used this wouldn't change
    //        const CharStringMapping::T2_value_type& rV = *itr;
    //        std::cout << "T2 # " << std::distance(c2s.T2_begin(), itr) << " = " << rV << "\n";
    //        ++itr;
    //    }
    //
template < typename T1, typename T2 >
class TwoWayMapping
{
public:
    //
    // Constructor
    // def1 and def2 will be returned by the convert method if no mapping exists
    //
    TwoWayMapping (T1 def1, T2 def2) :
        m_defaultType1(def1),
        m_defaultType2(def2) {}

    //
    // Add a mapping
    //
    void add(const T1& t1, const T2& t2)
    {
        m_Type2sToType1s[t2] = t1;
        m_Type1sToType2s[t1] = t2;
    }


    //
    // Check if a mapping exists
    //
    bool exists (const T1& e) const
    {
        return ( m_Type1sToType2s.find(e) != m_Type1sToType2s.end() );
    }
    bool exists (const T2& e) const
    {
        return ( m_Type2sToType1s.find(e) != m_Type2sToType1s.end() );
    }

    //
    // Get the mapping of T1 => T2
    //
    const T2& convert (const T1& e) const
    {
        typename T1toT2Mapping::const_iterator itr =m_Type1sToType2s.find(e);
        return ( itr != m_Type1sToType2s.end() )
               ? itr->second
               : m_defaultType2;
    }

    //
    // Get the mapping of T2 => T1
    //
    const T1& convert (const T2& e) const
    {
        typename T2toT1Mapping::const_iterator itr =m_Type2sToType1s.find(e);
        return ( itr != m_Type2sToType1s.end() )
               ? itr->second
               : m_defaultType1;
    }

    //
    // The data (stored are two maps)
    //
private:
    typedef std::map<T1, T2> T1toT2Mapping;
    typedef std::map<T2, T1> T2toT1Mapping;

    T1toT2Mapping m_Type1sToType2s;
    T2toT1Mapping m_Type2sToType1s;
    T1 m_defaultType1;
    T2 m_defaultType2;

    /////////////////////////////////////////////////////////////
    //
    // Now a load of complicated stuff to allow iteration
    //
    /////////////////////////////////////////////////////////////
public:
    //
    // Iterators types and values for T1 -> T2 and vice versa
    //
    typedef typename T1toT2Mapping::const_iterator const_T1toT2_iterator;
    typedef typename T2toT1Mapping::const_iterator const_T2toT1_iterator;

    typedef typename T1toT2Mapping::value_type T1toT2_value_type;
    typedef typename T2toT1Mapping::value_type T2toT1_value_type;

    //
    // Iterator types and values for iteratoring over only T1 and T2s
    //
    typedef iterator_to_member< const_T1toT2_iterator, T1toT2_value_type, const T1 > const_T1_iterator;
    typedef iterator_to_member< const_T2toT1_iterator, T2toT1_value_type, const T2 > const_T2_iterator;

    typedef T1 T1_value_type;
    typedef T2 T2_value_type;

    //
    // Size function (all are same size so only need one)
    //
    typename T1toT2Mapping::size_type size() { return m_Type1sToType2s.size(); }

    //
    // The begin/end's for T1<>T2 iterators
    //
    const_T1toT2_iterator T1toT2_begin() { return m_Type1sToType2s.begin(); }
    const_T1toT2_iterator T1toT2_end()   { return m_Type1sToType2s.end();  }

    const_T2toT1_iterator T2toT1_begin() { return m_Type2sToType1s.begin(); }
    const_T2toT1_iterator T2toT1_end()   { return m_Type2sToType1s.end();  }

    //
    // The begin/end's for T1 and T2 only iterators
    //
    const_T1_iterator T1_begin() { return const_T1_iterator( m_Type1sToType2s.begin(), &T1toT2_value_type::first); }
    const_T1_iterator T1_end()   { return const_T1_iterator( m_Type1sToType2s.end(), &T1toT2_value_type::first); }

    const_T2_iterator T2_begin() { return const_T2_iterator( m_Type2sToType1s.begin(), &T2toT1_value_type::first); }
    const_T2_iterator T2_end()   { return const_T2_iterator( m_Type2sToType1s.end(), &T2toT1_value_type::first); }
};

} // namespace

#endif

