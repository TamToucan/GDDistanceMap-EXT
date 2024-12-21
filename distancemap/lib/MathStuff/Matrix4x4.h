#ifndef MATHSTUFF_MATRIX4x4_H
#define MATHSTUFF_MATRIX4x4_H

#include <assert.h>
#include "MathUtils.h"
#include "Vector3.h"

namespace MathStuff {

template<typename VT> class Vector3;

template<typename MT>
class Matrix4x4
{
public:
    // Constructor
    // IF loadIde set then matrix is init'ed
    // with the identity matrix.
    Matrix4x4(bool loadId=true) {
        if (loadId)
        {
            loadIdentity();
        }
    }

    // Constructor from arrat of 16 values
    Matrix4x4(const MT* mtx) {
        memcpy(&m_mtx[0], mtx, sizeof(m_mtx));
    }

    // Copy Constructor
    Matrix4x4(const Matrix4x4<MT>& m) {
        clone(m);
    }

    // Destructor
    virtual ~Matrix4x4() { }

    // Return the start of the matrix so can use in gl funcs
    MT* getMatrixPtr() {
        return &m_mtx[0];
    }

    // Return the start of the matrix so can use in gl funcs
    const MT* getMatrixPtr() const {
        return &m_mtx[0];
    }

    // Accessor
    MT& operator[](int idx) {
        assert(idx < 16);
        return m_mtx[idx];
    }

    // Const Accessor
    const MT& operator[](int idx) const {
        assert(idx < 16);
        return m_mtx[idx];
    }

    // Assignment operator
    Matrix4x4<MT>& operator=(const Matrix4x4<MT>& m) {
        clone(m);
        return *this;
    }

    // Init the 4x4 matrix with the identity matrix
    void loadIdentity() {
        m_mtx[0] = (MT) 1;
        m_mtx[1] = m_mtx[2] = m_mtx[3] = m_mtx[4] = (MT) 0;
        m_mtx[5] = (MT) 1;
        m_mtx[6] = m_mtx[7] = m_mtx[8] = m_mtx[9] = (MT) 0;
        m_mtx[10] = (MT) 1;
        m_mtx[11] = m_mtx[12] = m_mtx[13] = m_mtx[14] = (MT) 0;
        m_mtx[15] = (MT) 1;
    }

    // Init the 4x4 matrix with a rotation matrix
    void loadRotation(float degrees, Vector3<MT> pos) {
        const MT l_radians = (MT) DEG_TO_RAD(degrees);
        const MT l_sin = (MT) sin(l_radians);
        const MT l_cos = (MT) cos(l_radians);
        const MT l_1minusCos = ((MT) 1) - l_cos;
        pos.normalize();

        const MT l_xx = pos[0]*pos[0];
        const MT l_yy = pos[1]*pos[1];
        const MT l_zz = pos[2]*pos[2];

        const MT l_xy = pos[0]*pos[1];
        const MT l_yz = pos[1]*pos[2];
        const MT l_zx = pos[2]*pos[0];

        const MT l_xs = pos[0]*l_sin;
        const MT l_ys = pos[1]*l_sin;
        const MT l_zs = pos[2]*l_sin;

        m_mtx[0] = (l_1minusCos * l_xx) + l_cos;
        m_mtx[4] = (l_1minusCos * l_xy) - l_zs;
        m_mtx[8] = (l_1minusCos * l_zx) + l_ys;
        m_mtx[12] = (MT) 0;

        m_mtx[1] = (l_1minusCos * l_xy) + l_zs;
        m_mtx[5] = (l_1minusCos * l_yy) + l_cos;
        m_mtx[9] = (l_1minusCos * l_yz) - l_xs;
        m_mtx[13] = (MT) 0;

        m_mtx[2] = (l_1minusCos * l_zx) - l_ys;
        m_mtx[6] = (l_1minusCos * l_yz) + l_xs;
        m_mtx[10] = (l_1minusCos * l_zz) + l_cos;
        m_mtx[14] = (MT) 0;

        m_mtx[3] = 0.0f;
        m_mtx[7] = 0.0f;
        m_mtx[11] = 0.0f;
        m_mtx[15] = (MT) 1;
    }

protected:
    // Make 'this' a clone of the input
    void clone(const Matrix4x4<MT>& m) {
        memcpy(&m_mtx[0], &m.m_mtx[0], sizeof(m_mtx));
    }

private:
    MT m_mtx[16];

};

// Matrix Multiplication operator
template<typename MT>
Matrix4x4<MT> operator*(const Matrix4x4<MT>& lhs, const Matrix4x4<MT>& rhs) {
    // Going to assign every value so don't bother to init the return matrix
    Matrix4x4<MT> l_retM(false);

    l_retM[0]  = lhs[0]*rhs[0]  + lhs[4]*rhs[1]  + lhs[8]*rhs[2]  + lhs[12]*rhs[3];
    l_retM[4]  = lhs[0]*rhs[4]  + lhs[4]*rhs[5]  + lhs[8]*rhs[6]  + lhs[12]*rhs[7];
    l_retM[8]  = lhs[0]*rhs[8]  + lhs[4]*rhs[9]  + lhs[8]*rhs[10] + lhs[12]*rhs[11];
    l_retM[12] = lhs[0]*rhs[12] + lhs[4]*rhs[13] + lhs[8]*rhs[14] + lhs[12]*rhs[15];

    l_retM[1]  = lhs[1]*rhs[0]  + lhs[5]*rhs[1]  + lhs[9]*rhs[2]  + lhs[13]*rhs[3];
    l_retM[5]  = lhs[1]*rhs[4]  + lhs[5]*rhs[5]  + lhs[9]*rhs[6]  + lhs[13]*rhs[7];
    l_retM[9]  = lhs[1]*rhs[8]  + lhs[5]*rhs[9]  + lhs[9]*rhs[10] + lhs[13]*rhs[11];
    l_retM[13] = lhs[1]*rhs[12] + lhs[5]*rhs[13] + lhs[9]*rhs[14] + lhs[13]*rhs[15];

    l_retM[2]  = lhs[2]*rhs[0]  + lhs[6]*rhs[1]  + lhs[10]*rhs[2]  + lhs[14]*rhs[3];
    l_retM[6]  = lhs[2]*rhs[4]  + lhs[6]*rhs[5]  + lhs[10]*rhs[6]  + lhs[14]*rhs[7];
    l_retM[10] = lhs[2]*rhs[8]  + lhs[6]*rhs[9]  + lhs[10]*rhs[10] + lhs[14]*rhs[11];
    l_retM[14] = lhs[2]*rhs[12] + lhs[6]*rhs[13] + lhs[10]*rhs[14] + lhs[14]*rhs[15];

    l_retM[3]  = lhs[3]*rhs[0]  + lhs[7]*rhs[1]  + lhs[11]*rhs[2]  + lhs[15]*rhs[3];
    l_retM[7]  = lhs[3]*rhs[4]  + lhs[7]*rhs[5]  + lhs[11]*rhs[6]  + lhs[15]*rhs[7];
    l_retM[11] = lhs[3]*rhs[8]  + lhs[7]*rhs[9]  + lhs[11]*rhs[10] + lhs[15]*rhs[11];
    l_retM[15] = lhs[3]*rhs[12] + lhs[7]*rhs[13] + lhs[11]*rhs[14] + lhs[15]*rhs[15];

    return l_retM;
}

template<typename MT>
std::ostream& operator<<(std::ostream& rOS, const MathStuff::Matrix4x4<MT>& rM)
{
    for (int i=0; i < 4; ++i)
    {
        for (int j=0; j < 4; ++j)
        {
            rOS << rM[i+j*4] << (j==3 ? '\n' : ':');
        }
    }
    return rOS;
}

} // namespace MathStuff


#endif
