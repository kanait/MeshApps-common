////////////////////////////////////////////////////////////////////
//
// Ray-triangle intersection (Eigen port of raytri.c)
//
// Original algorithm by Tomas Akenine-Möller (JGT / raytri).
// C++ / Eigen wrapper (replaces raytri.c).
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
// Algorithm portions Copyright 2020 Tomas Akenine-Möller (MIT license).
//
////////////////////////////////////////////////////////////////////

#ifndef _RAYTRI_HXX
#define _RAYTRI_HXX 1

#include <array>
#include <cmath>

#include "myEigen.hxx"

template <typename Scalar>
struct RayTriangleHitT {
  bool hit = false;
  Scalar t = Scalar(0);
  Scalar u = Scalar(0);
  Scalar v = Scalar(0);
};

using RayTriangleHit = RayTriangleHitT<double>;
using RayTriangleHitf = RayTriangleHitT<float>;

// Moller–Trumbore style ray/triangle tests (raytri.c variants).
class RayTriangleIntersection {
 public:
  static constexpr double kEpsilon = 1.0e-6;

  template <typename Scalar>
  static RayTriangleHitT<Scalar> intersect(const Eigen::Matrix<Scalar, 3, 1>& orig,
                                           const Eigen::Matrix<Scalar, 3, 1>& dir,
                                           const Eigen::Matrix<Scalar, 3, 1>& v0,
                                           const Eigen::Matrix<Scalar, 3, 1>& v1,
                                           const Eigen::Matrix<Scalar, 3, 1>& v2,
                                           int algorithm = 1) {
    RayTriangleHitT<Scalar> result;
    switch (algorithm) {
      case 0:
        result.hit =
            intersectOriginal(orig, dir, v0, v1, v2, result.t, result.u, result.v);
        break;
      case 2:
        result.hit =
            intersectEarlyInv(orig, dir, v0, v1, v2, result.t, result.u, result.v);
        break;
      case 3:
        result.hit =
            intersectHoistCross(orig, dir, v0, v1, v2, result.t, result.u, result.v);
        break;
      case 1:
      default:
        result.hit =
            intersectSignDet(orig, dir, v0, v1, v2, result.t, result.u, result.v);
        break;
    }
    return result;
  }

  static RayTriangleHit intersect(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
                                  const Eigen::Vector3d& v0, const Eigen::Vector3d& v1,
                                  const Eigen::Vector3d& v2, int algorithm = 1) {
    return intersect<double>(orig, dir, v0, v1, v2, algorithm);
  }

  static RayTriangleHitf intersect(const Eigen::Vector3f& orig, const Eigen::Vector3f& dir,
                                   const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
                                   const Eigen::Vector3f& v2, int algorithm = 1) {
    return intersect<float>(orig, dir, v0, v1, v2, algorithm);
  }

  static RayTriangleHit intersect(const Eigen::Vector3d& orig, const Eigen::Vector3d& dir,
                                  const std::array<Eigen::Vector3d, 3>& triangle,
                                  int algorithm = 1) {
    return intersect(orig, dir, triangle[0], triangle[1], triangle[2], algorithm);
  }

  static RayTriangleHitf intersect(const Eigen::Vector3f& orig, const Eigen::Vector3f& dir,
                                   const std::array<Eigen::Vector3f, 3>& triangle,
                                   int algorithm = 1) {
    return intersect(orig, dir, triangle[0], triangle[1], triangle[2], algorithm);
  }

  // intersect_triangle — original JGT code.
  template <typename Scalar>
  static bool intersectOriginal(const Eigen::Matrix<Scalar, 3, 1>& orig,
                                const Eigen::Matrix<Scalar, 3, 1>& dir,
                                const Eigen::Matrix<Scalar, 3, 1>& v0,
                                const Eigen::Matrix<Scalar, 3, 1>& v1,
                                const Eigen::Matrix<Scalar, 3, 1>& v2, Scalar& t, Scalar& u,
                                Scalar& v) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    const Scalar eps = epsilon<Scalar>();

    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 pvec = dir.cross(edge2);
    const Scalar det = edge1.dot(pvec);

    if (det > -eps && det < eps) return false;
    const Scalar inv_det = Scalar(1) / det;

    const Vec3 tvec = orig - v0;
    u = tvec.dot(pvec) * inv_det;
    if (u < Scalar(0) || u > Scalar(1)) return false;

    const Vec3 qvec = tvec.cross(edge1);
    v = dir.dot(qvec) * inv_det;
    if (v < Scalar(0) || u + v > Scalar(1)) return false;

    t = edge2.dot(qvec) * inv_det;
    return true;
  }

  // intersect_triangle1 — sign of determinant tested before division (recommended).
  template <typename Scalar>
  static bool intersectSignDet(const Eigen::Matrix<Scalar, 3, 1>& orig,
                               const Eigen::Matrix<Scalar, 3, 1>& dir,
                               const Eigen::Matrix<Scalar, 3, 1>& v0,
                               const Eigen::Matrix<Scalar, 3, 1>& v1,
                               const Eigen::Matrix<Scalar, 3, 1>& v2, Scalar& t, Scalar& u,
                               Scalar& v) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    const Scalar eps = epsilon<Scalar>();

    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 pvec = dir.cross(edge2);
    const Scalar det = edge1.dot(pvec);

    Vec3 qvec;
    if (det > eps) {
      const Vec3 tvec = orig - v0;
      u = tvec.dot(pvec);
      if (u < Scalar(0) || u > det) return false;
      qvec = tvec.cross(edge1);
      v = dir.dot(qvec);
      if (v < Scalar(0) || u + v > det) return false;
    } else if (det < -eps) {
      const Vec3 tvec = orig - v0;
      u = tvec.dot(pvec);
      if (u > Scalar(0) || u < det) return false;
      qvec = tvec.cross(edge1);
      v = dir.dot(qvec);
      if (v > Scalar(0) || u + v < det) return false;
    } else {
      return false;
    }

    const Scalar inv_det = Scalar(1) / det;
    t = edge2.dot(qvec) * inv_det;
    u *= inv_det;
    v *= inv_det;
    return true;
  }

  // intersect_triangle2 — used by Octree.hxx today.
  template <typename Scalar>
  static bool intersectEarlyInv(const Eigen::Matrix<Scalar, 3, 1>& orig,
                                const Eigen::Matrix<Scalar, 3, 1>& dir,
                                const Eigen::Matrix<Scalar, 3, 1>& v0,
                                const Eigen::Matrix<Scalar, 3, 1>& v1,
                                const Eigen::Matrix<Scalar, 3, 1>& v2, Scalar& t, Scalar& u,
                                Scalar& v) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    const Scalar eps = epsilon<Scalar>();

    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 pvec = dir.cross(edge2);
    const Scalar det = edge1.dot(pvec);
    const Vec3 tvec = orig - v0;
    const Scalar inv_det = Scalar(1) / det;

    Vec3 qvec;
    if (det > eps) {
      u = tvec.dot(pvec);
      if (u < Scalar(0) || u > det) return false;
      qvec = tvec.cross(edge1);
      v = dir.dot(qvec);
      if (v < Scalar(0) || u + v > det) return false;
    } else if (det < -eps) {
      u = tvec.dot(pvec);
      if (u > Scalar(0) || u < det) return false;
      qvec = tvec.cross(edge1);
      v = dir.dot(qvec);
      if (v > Scalar(0) || u + v < det) return false;
    } else {
      return false;
    }

    t = edge2.dot(qvec) * inv_det;
    u *= inv_det;
    v *= inv_det;
    return true;
  }

  // intersect_triangle3 — hoisted cross product.
  template <typename Scalar>
  static bool intersectHoistCross(const Eigen::Matrix<Scalar, 3, 1>& orig,
                                  const Eigen::Matrix<Scalar, 3, 1>& dir,
                                  const Eigen::Matrix<Scalar, 3, 1>& v0,
                                  const Eigen::Matrix<Scalar, 3, 1>& v1,
                                  const Eigen::Matrix<Scalar, 3, 1>& v2, Scalar& t, Scalar& u,
                                  Scalar& v) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    const Scalar eps = epsilon<Scalar>();

    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 pvec = dir.cross(edge2);
    const Scalar det = edge1.dot(pvec);
    const Vec3 tvec = orig - v0;
    const Scalar inv_det = Scalar(1) / det;
    const Vec3 qvec = tvec.cross(edge1);

    if (det > eps) {
      u = tvec.dot(pvec);
      if (u < Scalar(0) || u > det) return false;
      v = dir.dot(qvec);
      if (v < Scalar(0) || u + v > det) return false;
    } else if (det < -eps) {
      u = tvec.dot(pvec);
      if (u > Scalar(0) || u < det) return false;
      v = dir.dot(qvec);
      if (v > Scalar(0) || u + v < det) return false;
    } else {
      return false;
    }

    t = edge2.dot(qvec) * inv_det;
    u *= inv_det;
    v *= inv_det;
    return true;
  }

  // Barycentric hit point: (1-u-v)*v0 + u*v1 + v*v2
  template <typename Scalar>
  static Eigen::Matrix<Scalar, 3, 1> hitPoint(const Eigen::Matrix<Scalar, 3, 1>& v0,
                                              const Eigen::Matrix<Scalar, 3, 1>& v1,
                                              const Eigen::Matrix<Scalar, 3, 1>& v2, Scalar u,
                                              Scalar v) {
    return (Scalar(1) - u - v) * v0 + u * v1 + v * v2;
  }

 private:
  template <typename Scalar>
  static Scalar epsilon() {
    return Scalar(kEpsilon);
  }
};

#endif  // _RAYTRI_HXX
