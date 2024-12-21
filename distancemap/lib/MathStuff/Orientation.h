#ifndef MATHSTUFF_ORIENTATION_H
#define MATHSTUFF_ORIENTATION_H

#include <assert.h>
#include <iostream>

#include "Matrix4x4.h"
#include "Vector3.h"

namespace MathStuff {

template<typename VT>
class Orientation
{
public:
    // Default Constructor (Postion = origin, Facing = +ve Z, Up = +ve Y)
    // The +ve Z  is so objects face the camera rather than have to turnLeft(180)
    // It therefore means the orientation for the camera needs a facing of 0,0,-1
    // (or turnLeft(180)
    Orientation() :
        m_vPos(0, 0, 0), m_vFace(0, 0, 1), m_vUp(0, 1, 0), m_vScale(1,1,1) { }

    // Construct from just Position (Facing = +ve Z, Up = +ve Y)
    Orientation(const Vector3<VT>& vPos) :
        m_vPos(vPos), m_vFace(0, 0, 1), m_vUp(0, 1, 0), m_vScale(1,1,1) { }

    // Construct from just Position and Facing (Up = +ve Y)
    Orientation(const Vector3<VT>& vPos, const Vector3<VT>& vFace) :
        m_vPos(vPos), m_vFace(vFace), m_vUp(0, 1, 0), m_vScale(1,1,1) { }

    // Construct from Position, Facing and Up vectors
    Orientation(const Vector3<VT>& vPos, const Vector3<VT>& vFace, const Vector3<VT>& vUp) :
        m_vPos(vPos), m_vFace(vFace), m_vUp(vUp), m_vScale(1,1,1) { }

    // Copy Constructor
    Orientation(const Orientation<VT>& o) {
        clone(o);
    }

    // Destructor NOTE: NON-virtual
    ~Orientation() { }

    // Assignment operator
    Orientation& operator=(const Orientation& o) {
        clone(o);
        return *this;
    }

    // Populate "transformation" matrix from the orientation
    void getTransformation(Matrix4x4<VT>& oMtx) const {
        // The X column (0,1,2,3) is the crossProduct of the
        // Up and Facing vectors i.e. the vector which is
        // perpendicular to them (the X-axis)
        Vector3<VT> l_xAxis(crossProduct(m_vUp, m_vFace));
        oMtx[0] = l_xAxis[0] * m_vScale[0];
        oMtx[1] = l_xAxis[1] * m_vScale[0];
        oMtx[2] = l_xAxis[2] * m_vScale[0];
        oMtx[3] = (VT) 0;

        // The Y column (4,5,6,7) is the Up vector
        oMtx[4] = m_vUp[0] * m_vScale[1];
        oMtx[5] = m_vUp[1] * m_vScale[1];
        oMtx[6] = m_vUp[2] * m_vScale[1];
        oMtx[7] = (VT) 0;

        // The Z column (8,9,10,11) is the Facing vector
        oMtx[8] = m_vFace[0] * m_vScale[2];
        oMtx[9] = m_vFace[1] * m_vScale[2];
        oMtx[10] = m_vFace[2] * m_vScale[2];
        oMtx[11] = (VT) 0;

        // The Location column (12,13,14,15) is the position
        oMtx[12] = m_vPos[0];
        oMtx[13] = m_vPos[1];
        oMtx[14] = m_vPos[2];
        oMtx[15] = (VT) 1;
    }

    // Populate "look at" matrix from the orientation
    // This is the "transformation" matrix above only
    // transposed i.e. populating rows instead of columns
    void getLookAt(Matrix4x4<VT>& oMtx) const {
        Vector3<VT> l_xAxis(crossProduct(m_vUp, -m_vFace));

        oMtx[0] = l_xAxis[0];
        oMtx[4] = l_xAxis[1];
        oMtx[8] = l_xAxis[2];
        oMtx[12] = (VT) 0;

        oMtx[1] = m_vUp[0];
        oMtx[5] = m_vUp[1];
        oMtx[9] = m_vUp[2];
        oMtx[13] = (VT) 0;

        oMtx[2] = -m_vFace[0];
        oMtx[6] = -m_vFace[1];
        oMtx[10] = -m_vFace[2];
        oMtx[14] = (VT) 0;

        oMtx[3] = (VT) 0;
        oMtx[7] = (VT) 0;
        oMtx[11] = (VT) 0;
        oMtx[15] = (VT) 1;
    }

    // Accessors
    const Vector3<VT>& getPos() const { return m_vPos; }
    const Vector3<VT>& getFace() const { return m_vFace; }
    const Vector3<VT>& getUp() const { return m_vUp; }
    const Vector3<VT>& getScale() const { return m_vScale; }

    // Acccessor for so can change position
    void setUp(const Vector3<VT>& rNewUp) { m_vUp = rNewUp; }
    void setFace(const Vector3<VT>& rNewFace) { m_vFace = rNewFace; }
    void setPos(const Vector3<VT>& rNewPos) { m_vPos = rNewPos; }
    void movePos(const Vector3<VT>& rMoveAmt) { m_vPos += rMoveAmt; }
    void setScale(const Vector3<VT>& rNewScale) { m_vScale = rNewScale; }

    void turnRight(float degrees) { turnLeft(-degrees); }
    void turnLeft(float degrees) {
        Matrix4x4<VT> l_mRot(false);
        l_mRot.loadRotation(degrees, m_vUp);
        m_vFace.rotate(l_mRot);
    }

    void tiltLeft(float degrees) { tiltRight(-degrees); }
    void tiltRight(float degrees) {
        Matrix4x4<VT> l_mRot(false);
        l_mRot.loadRotation(degrees, m_vFace);
        m_vUp.rotate(l_mRot);
    }

    void lookUp(float degrees) { lookDown(-degrees); }
    void lookDown(float degrees) {
        // X-axis is perpendicular to the Up and Facing vectors
        Vector3<VT> l_xAxis(crossProduct(m_vUp, m_vFace));
        Matrix4x4<VT> l_mRot(false);
        l_mRot.loadRotation(degrees, l_xAxis);
        m_vUp.rotate(l_mRot);
        m_vFace.rotate(l_mRot);
    }

    void moveDown(VT amount) { moveUp(-amount); }
    void moveUp(VT amount) {
        m_vPos += m_vUp * amount;
    }

    void moveRight(VT amount) { moveLeft(-amount); }
    void moveLeft(VT amount) {
        Vector3<VT> l_xAxis(crossProduct(m_vUp, m_vFace));
        m_vPos += l_xAxis * amount;
    }

    void moveBackward(VT amount) { moveForward(-amount); }
    void moveForward(VT amount) {
        m_vPos += m_vFace * amount;
    }

protected:
    // Make 'this' a clone of the input
    void clone(const Orientation<VT>& o) {
        m_vPos = o.m_vPos;
        m_vFace = o.m_vFace;
        m_vUp = o.m_vUp;
        m_vScale = o.m_vScale;
    }

private:
    // Position, direction facing and direction of up
    Vector3<VT> m_vPos;
    Vector3<VT> m_vFace;
    Vector3<VT> m_vUp;
    Vector3<VT> m_vScale;
};

template<typename VT>
std::ostream& operator<<(std::ostream& rOS, const Orientation<VT>& rO)
{
    return rOS << "POS:" << rO.getPos() << " FAC:" << rO.getFace() << " UP: " << rO.getUp() << std::endl;
}

template<typename VT>
std::istream& operator>>(std::istream& rIS, Orientation<VT>& rO)
{
    Vector3<VT> l_newP;
    Vector3<VT> l_newF;
    Vector3<VT> l_newU;
    rIS.ignore(10,':'); // POS:
    rIS >> l_newP;
    rIS.ignore(10,':'); // FAC:
    rIS >> l_newF;
    rIS.ignore(10,':'); // UP:
    rIS >> l_newU;
    rIS.ignore(10,'\n');
    rO.setUp(l_newU);
    rO.setFace(l_newF);
    rO.setPos(l_newP);
    return rIS;
}

} // namespace MathStuff


#endif
