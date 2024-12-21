#ifndef UTIL_STL_STUFF

#include <utility>
#include <functional>

namespace Util {

//
// Compare value against the .first member of the pair for equality
// Use like this
//   l_found = std::find_if(m_vector.begin(), m_vector.end(),
//				std::bind2nd( Util::first_equals<int, std::string>(), iVal) );
//
template<typename PT1, typename PT2>
struct first_equals : std::binary_function< std::pair<PT1,PT2>, PT1, bool> {
    bool operator()(const std::pair<PT1, PT2>& p, const PT1& f) const {
        return p.first == f;
    }
};

//
// As above, but for .second member
//
template<typename PT1, typename PT2>
struct second_equals : std::binary_function< std::pair<PT1,PT2>, PT2, bool> {
    bool operator()(const std::pair<PT1, PT2>& p, const PT2& s) const {
        return p.second == s;
    }
};

//////////////////////////////////////////////////////

//
// As previous, but takes a std::pair
//
template<typename PT>
struct pair_first_equals : std::binary_function< PT, typename PT::first_type, bool> {
    bool operator()(const PT& p, const typename PT::first_type& f) const {
        return p.first == f;
    }
};

//
// As previous, but takes a std::pair
//
template<typename PT>
struct pair_second_equals : std::binary_function< PT, typename PT::second_type, bool> {
    bool operator()(const PT& p, const typename PT::second_type& f) const {
        return p.second == f;
    }
};

//////////////////////////////////////////////////////


//
// Equivalent to the SGI select1st
//
template<typename PT>
struct select1st : public std::unary_function<PT, typename PT::first_type>
{
    typename PT::first_type& operator()(PT& o) const { return o.first; }
    const typename PT::first_type& operator()(const PT& o) const { return o.first; }
};

//
// As above, but for select2nd
template<typename PT>
struct select2nd : public std::unary_function<PT, typename PT::second_type>
{
    typename PT::second_type& operator()(PT& o) const { return o.second; }
    const typename PT::second_type& operator()(const PT& o) const { return o.second; }
};

//////////////////////////////////////////////////////

//
// compose two objects so that the result of the 2nd is passed to the first
//
template <class OP1, class OP2>
class unary_compose : public std::unary_function<typename OP2::argument_type, typename OP1::result_type>
{
protected:
    OP1 fn1;
    OP2 fn2;

public:
    unary_compose(const OP1& x, const OP2& y) : fn1(x), fn2(y) {}

    typename OP1::result_type
    operator()(const typename OP2::argument_type& x) const { return fn1(fn2(x)); }
};

// Helper function
template <class OP1, class OP2>
inline unary_compose<OP1, OP2>
compose1(const OP1& fn1, const OP2& fn2) { return unary_compose<OP1,OP2>(fn1, fn2); }


} // namespace

#endif
