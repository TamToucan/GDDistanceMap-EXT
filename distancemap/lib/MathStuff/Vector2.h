#ifndef MATHSTUFF_VECTOR2_H
#define MATHSTUFF_VECTOR2_H

#include <assert.h>
#include <string.h>
#include <iostream>

#include "MathUtils.h"
#include "Matrix4x4.h"

namespace MathStuff {

template<typename MT> class Matrix4x4;

template<typename VT>
class Vector2
{
public:
    // Default Constructor
    Vector2()
    {
        memset(&m_vec[0], 0x00, sizeof(m_vec));
    }

    // Constructor with 3 values
    Vector2(VT i, VT j)
    {
        m_vec[0] = i;
        m_vec[1] = j;
    }

    // Copy Constructor
    Vector2(const Vector2<VT>& v)
    {
        clone(v);
    }

    // Constructor from another Vector type
    template<typename VT2>
    Vector2(const Vector2<VT2>& v)
    {
        m_vec[0] = VT(v[0]);
        m_vec[1] = VT(v[1]);
    }

    // Destructor
    ~Vector2() { }

    // Accessor
    VT& operator[](int idx) {
        assert(idx < 2);
        return m_vec[idx];
    }

    // Const Accessor:
    const VT& operator[](int idx) const {
        assert(idx < 2);
        return m_vec[idx];
    }

    // Length of the vector
    // Use the ANSI-C++ float verstion since probably using GLfloat
    VT length() const {
        return (VT) sqrt((float) (m_vec[0]*m_vec[0]) + (m_vec[1]*m_vec[1]) );
    }

    // Y Angle of vector
    // NOTE: Assumes vector has been normalized!
    VT xangle() const {
        return (VT) RAD_TO_DEG(asin(m_vec[0]));
    }

    // Y Angle of vector
    // NOTE: Assumes vector has been normalized!
    VT yangle() const {
        return (VT) RAD_TO_DEG(asin(m_vec[1]));
    }

    // Normalize the vector
    // i.e. scale (multiply) it by 1 over it's length
    void normalize() {
        *this *= ( ((VT) 1) / length());
    }

    // Rotate the vector using the Matrix
    void rotate(const Matrix4x4<VT>& m) {
        Vector2<VT> l_vNew;
        l_vNew[0] = (m[0]*m_vec[0]) + (m[4]*m_vec[1]) + (m[8]*m_vec[2]);
        l_vNew[1] = (m[1]*m_vec[0]) + (m[5]*m_vec[1]) + (m[9]*m_vec[2]);
        clone(l_vNew);
    }
    // Rotate and Translate the vector using the Matrix
    void rotateAndTrans(const Matrix4x4<VT>& m) {
        Vector2<VT> l_vNew;
        l_vNew[0] = (m[0]*m_vec[0]) + (m[4]*m_vec[1]) + (m[8]*m_vec[2]) + m[12];
        l_vNew[1] = (m[1]*m_vec[0]) + (m[5]*m_vec[1]) + (m[9]*m_vec[2]) + m[13];
        clone(l_vNew);
    }

    // The Dot product
    VT dotProduct(const Vector2<VT>& v) const {
        return (m_vec[0]*v.m_vec[0]) + (m_vec[1]*v.m_vec[1]);
    }

    // Assignment operator
    Vector2<VT>& operator=(const Vector2<VT>& v) {
        clone(v);
        return *this;
    }

    // Helper *= operator
    void operator*=(VT scalar) {
        m_vec[0] *= scalar;
        m_vec[1] *= scalar;
    }

    // Helper /= operator
    void operator/=(VT scalar) {
        m_vec[0] /= scalar;
        m_vec[1] /= scalar;
    }

    // Helper -= operator
    void operator-=(const Vector2<VT>& v) {
        m_vec[0] -= v.m_vec[0];
        m_vec[1] -= v.m_vec[1];
    }

    // Helper += operator
    void operator+=(const Vector2<VT>& v) {
        m_vec[0] += v.m_vec[0];
        m_vec[1] += v.m_vec[1];
    }


protected:
    // Make 'this' a clone of the input
    void clone(const Vector2<VT>& v) {
        m_vec[0] = v.m_vec[0];
        m_vec[1] = v.m_vec[1];
    }

private:
    VT m_vec[2];
};

// operator ==
template <typename VT>
bool operator==(const Vector2<VT>& lhs, const Vector2<VT>& rhs) {
    return (    (lhs[0] == lhs[0])
            and (lhs[1] == lhs[1]) );
}
// operator == for FLOAT
template <>
inline
bool operator==(const Vector2<float>& lhs, const Vector2<float>& rhs) {
    return (    (FLOATS_EQUAL(lhs[0],rhs[0]))
            and (FLOATS_EQUAL(lhs[1],rhs[1])) );
}


// The cross product
template<typename VT>
VT crossProduct(const Vector2<VT>& lhs, const Vector2<VT>& rhs) {
    return (lhs[0]*rhs[1]) - (lhs[1]*rhs[0]);
}

// Helper * operator
template<typename VT>
Vector2<VT> operator*(VT scalar, const Vector2<VT>& v) { return v*scalar; }
template<typename VT>
Vector2<VT> operator*(const Vector2<VT>& v, VT scalar) {
    Vector2<VT> l_retV(v[0]*scalar, v[1]*scalar);
    return l_retV;
}


// Helper + operator
template<typename VT>
Vector2<VT> operator+(const Vector2<VT>& lhs, const Vector2<VT>& rhs) {
    Vector2<VT> l_retV(lhs[0]+rhs[0], lhs[1]+rhs[1]);
    return l_retV;
}

// Helper - operator
template<typename VT>
Vector2<VT> operator-(const Vector2<VT>& lhs, const Vector2<VT>& rhs) {
    Vector2<VT> l_retV(lhs[0]-rhs[0], lhs[1]-rhs[1]);
    return l_retV;
}

// Helper unary - operator
template<typename VT>
Vector2<VT> operator-(const Vector2<VT>& v) {
    Vector2<VT> l_retV(-v[0], -v[1]);
    return l_retV;
}


// Helper cout (for debug)
template<typename VT>
std::ostream& operator<<(std::ostream& os, const MathStuff::Vector2<VT>& v) {
    return os << v[0] << ':' << v[1];
}

template<typename VT>
std::istream& operator>>(std::istream& is, MathStuff::Vector2<VT>& v) {
    char c;
    return is >> v[0] >> c >> v[1];
}

} // namespace MathStuff


#endif
