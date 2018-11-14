#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

 RigTForm(const Cvec3& t, const Quat& r) : t_(t), r_(r) {
  }

  explicit RigTForm(const Cvec3& t) : t_(t) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  explicit RigTForm(const Quat& r) : t_(0), r_(r) {
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
     return r_ * a + Cvec4(t_[0], t_[1], t_[2], 0.0);
  }

  RigTForm operator * (const RigTForm& a) const {
     
     const Cvec3 t2 = a.getTranslation();
     const Quat r2 = a.getRotation();
     const Cvec4 tNew = Cvec4(t_[0], t_[1], t_[2], 0.0)
        + r_ * Cvec4(t2[0], t2[1], t2[2], 0.0);
     const Quat rNew = r_ * r2;
     return RigTForm(Cvec3(tNew[0], tNew[1], tNew[2]), rNew);
  }

  static RigTForm identity() {
     Quat r = Quat::makeXRotation(0.0);
     return RigTForm(Cvec3(0.0,0.0,0.0), r);
  }

  static RigTForm makeXRotation(const double ang) {
     Quat r = Quat::makeXRotation(ang);
     return RigTForm(Cvec3(0.0,0.0,0.0), r);
  }

  static RigTForm makeYRotation(const double ang) {
     Quat r = Quat::makeYRotation(ang);
     return RigTForm(Cvec3(0.0,0.0,0.0), r);
  }

  static RigTForm makeZRotation(const double ang) {
     Quat r = Quat::makeZRotation(ang);
     return RigTForm(Cvec3(0.0,0.0,0.0), r);
  }

  static RigTForm makeTranslation(const Cvec3& t) {
     return RigTForm(t);
  }
};

inline RigTForm inv(const RigTForm& tform) {
   const Cvec3 t = tform.getTranslation();
   const Quat r = tform.getRotation();
   const Quat rInv = inv(r);
   const Cvec4 tNew = -(rInv * Cvec4(t[0], t[1], t[2], 0.0));
   return RigTForm(Cvec3(tNew[0], tNew[1], tNew[2]), rInv);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 makeRotation(const Quat& r)
{
   Matrix4 R;
   Cvec3 k = Cvec3(r[1], r[2], r[3]);
   double kLen = std::sqrt(norm2(k));
   double theta = 2.0 * atan2(kLen, r[0]);

   if (fabs(theta) < CS175_EPS)
      return R;

   k = k / kLen;
   double kx = k[0];
   double ky = k[1];
   double kz = k[2];
   double c = std::cos(theta);
   double s = std::sin(theta);
   double v =  1.0 - c;
   R(0, 0) = kx * kx * v + c;
   R(0, 1) = kx * ky * v - kz * s;
   R(0, 2) = kx * kz * v + ky * s;
   R(1, 0) = ky * kx * v + kz * s;
   R(1, 1) = ky * ky * v + c;
   R(1, 2) = ky * kz * v - kx * s;
   R(2, 0) = kz * kx * v - ky * s;
   R(2, 1) = kz * ky * v + kx * s;
   R(2, 2) = kz * kz * v + c;
   return R;
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
   const Cvec3 t = tform.getTranslation();
   const Quat r = tform.getRotation();
   return  Matrix4::makeTranslation(t) * makeRotation(r);
}

inline RigTForm makeMixedFrame(const RigTForm& O, const RigTForm& E) {
   return transFact(O) * linFact(E);
}

inline RigTForm doMtoOwrtA(const RigTForm& M, const RigTForm& O,
                          const RigTForm& A) {
   return A * M * inv(A) * O;
}

#endif
