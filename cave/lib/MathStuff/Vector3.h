#ifndef MATHSTUFF_VECTOR3_H
#define MATHSTUFF_VECTOR3_H

#include <assert.h>
#include <string.h>
#include <iostream>

#include "MathUtils.h"
#include "Matrix4x4.h"

namespace MathStuff {

template<typename MT> class Matrix4x4;

template<typename VT>
class Vector3
{
public:
    // Default Constructor
    Vector3()
    {
        memset(&m_vec[0], 0x00, sizeof(m_vec));
    }

    // Constructor with 3 values
    Vector3(VT i, VT j, VT k)
    {
        m_vec[0] = i;
        m_vec[1] = j;
        m_vec[2] = k;
    }

    // Copy Constructor
    Vector3(const Vector3<VT>& v)
    {
        clone(v);
    }

    // Constructor from another Vector type
    template<typename VT2>
    Vector3(const Vector3<VT2>& v)
    {
        m_vec[0] = VT(v[0]);
        m_vec[1] = VT(v[1]);
        m_vec[2] = VT(v[2]);
    }

    // Destructor
    ~Vector3() { }

    // Accessor
    VT& operator[](int idx) {
        assert(idx < 3);
        return m_vec[idx];
    }

    // Const Accessor:
    const VT& operator[](int idx) const {
        assert(idx < 3);
        return m_vec[idx];
    }

    // Squared Length of the vector
    VT lengthSquared() const {
        return (m_vec[0]*m_vec[0]) + (m_vec[1]*m_vec[1]) + (m_vec[2]*m_vec[2]);
    }

    // Length of the vector
    // Use the ANSI-C++ float verstion since probably using GLfloat
    VT length() const {
        return (VT) sqrt((float) lengthSquared());
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

    // Z Angle of vector
    // NOTE: Assumes vector has been normalized!
    VT zangle() const {
        return (VT) RAD_TO_DEG(asin(m_vec[2]));
    }

    // Normalize the vector
    // i.e. scale (multiply) it by 1 over it's length
    void normalize() {
        const VT l = length();
        if (l) *this *= ( ((VT) 1) / l );
    }

    // Rotate the vector using the Matrix
    void rotate(const Matrix4x4<VT>& m) {
        Vector3<VT> l_vNew;
        l_vNew[0] = (m[0]*m_vec[0]) + (m[4]*m_vec[1]) + (m[8]*m_vec[2]);
        l_vNew[1] = (m[1]*m_vec[0]) + (m[5]*m_vec[1]) + (m[9]*m_vec[2]);
        l_vNew[2] = (m[2]*m_vec[0]) + (m[6]*m_vec[1]) + (m[10]*m_vec[2]);
        clone(l_vNew);
    }
    // Rotate and Translate the vector using the Matrix
    void rotateAndTrans(const Matrix4x4<VT>& m) {
        Vector3<VT> l_vNew;
        l_vNew[0] = (m[0]*m_vec[0]) + (m[4]*m_vec[1]) + (m[8]*m_vec[2]) + m[12];
        l_vNew[1] = (m[1]*m_vec[0]) + (m[5]*m_vec[1]) + (m[9]*m_vec[2]) + m[13];
        l_vNew[2] = (m[2]*m_vec[0]) + (m[6]*m_vec[1]) + (m[10]*m_vec[2]) + m[14];
        clone(l_vNew);
    }

    // The Dot product
    VT dotProduct(const Vector3<VT>& v) const {
        return (m_vec[0]*v.m_vec[0]) + (m_vec[1]*v.m_vec[1]) + (m_vec[2]*v.m_vec[2]);
    }

    // Assignment operator
    Vector3<VT>& operator=(const Vector3<VT>& v) {
        clone(v);
        return *this;
    }

    // Helper *= operator
    void operator*=(VT scalar) {
        m_vec[0] *= scalar;
        m_vec[1] *= scalar;
        m_vec[2] *= scalar;
    }

    // Helper /= operator
    void operator/=(VT scalar) {
        m_vec[0] /= scalar;
        m_vec[1] /= scalar;
        m_vec[2] /= scalar;
    }

    // Helper -= operator
    void operator-=(const Vector3<VT>& v) {
        m_vec[0] -= v.m_vec[0];
        m_vec[1] -= v.m_vec[1];
        m_vec[2] -= v.m_vec[2];
    }

    // Helper += operator
    void operator+=(const Vector3<VT>& v) {
        m_vec[0] += v.m_vec[0];
        m_vec[1] += v.m_vec[1];
        m_vec[2] += v.m_vec[2];
    }


protected:
    // Make 'this' a clone of the input
    void clone(const Vector3<VT>& v) {
        m_vec[0] = v.m_vec[0];
        m_vec[1] = v.m_vec[1];
        m_vec[2] = v.m_vec[2];
    }

private:
    VT m_vec[3];
};

// operator ==
template <typename VT>
bool operator==(const Vector3<VT>& lhs, const Vector3<VT>& rhs) {
    return (    (lhs[0] == rhs[0])
            and (lhs[1] == rhs[1])
            and (lhs[2] == rhs[2]) );
}
// operator == for FLOAT
template <>
inline
bool operator==(const Vector3<float>& lhs, const Vector3<float>& rhs) {
    return (    (FLOATS_EQUAL(lhs[0],rhs[0]))
            and (FLOATS_EQUAL(lhs[1],rhs[1]))
            and (FLOATS_EQUAL(lhs[2],rhs[2])) );
}


// The cross product
template<typename VT>
Vector3<VT> crossProduct(const Vector3<VT>& lhs, const Vector3<VT>& rhs) {
    Vector3<VT> l_retV;
    l_retV[0] = (lhs[1]*rhs[2]) - (lhs[2]*rhs[1]);
    l_retV[1] = (lhs[2]*rhs[0]) - (lhs[0]*rhs[2]);
    l_retV[2] = (lhs[0]*rhs[1]) - (lhs[1]*rhs[0]);
    return l_retV;
}

// Helper * operator
template<typename VT>
Vector3<VT> operator*(VT scalar, const Vector3<VT>& v) { return v*scalar; }
template<typename VT>
Vector3<VT> operator*(const Vector3<VT>& v, VT scalar) {
    Vector3<VT> l_retV(v[0]*scalar, v[1]*scalar, v[2]*scalar);
    return l_retV;
}

// Helper * operator
template<typename VT>
Vector3<VT> operator*(const Vector3<VT>& lhs, const Vector3<VT>& rhs) {
    Vector3<VT> l_retV(lhs[0]*rhs[0], lhs[1]*rhs[1], lhs[2]*rhs[2]);
    return l_retV;
}

// Helper + operator
template<typename VT>
Vector3<VT> operator+(const Vector3<VT>& lhs, const Vector3<VT>& rhs) {
    Vector3<VT> l_retV(lhs[0]+rhs[0], lhs[1]+rhs[1], lhs[2]+rhs[2]);
    return l_retV;
}

// Helper - operator
template<typename VT>
Vector3<VT> operator-(const Vector3<VT>& lhs, const Vector3<VT>& rhs) {
    Vector3<VT> l_retV(lhs[0]-rhs[0], lhs[1]-rhs[1], lhs[2]-rhs[2]);
    return l_retV;
}

// Helper unary - operator
template<typename VT>
Vector3<VT> operator-(const Vector3<VT>& v) {
    Vector3<VT> l_retV(-v[0], -v[1], -v[2]);
    return l_retV;
}


// Helper cout (for debug)
template<typename VT>
std::ostream& operator<<(std::ostream& os, const MathStuff::Vector3<VT>& v) {
    return os << v[0] << ':' << v[1] << ':' << v[2];
}

template<typename VT>
std::istream& operator>>(std::istream& is, MathStuff::Vector3<VT>& v) {
    char c;
    return is >> v[0] >> c >> v[1] >> c >> v[2];
}

} // namespace MathStuff


#endif
