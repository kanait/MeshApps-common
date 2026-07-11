////////////////////////////////////////////////////////////////////
//
// AABB-triangle overlap test (Eigen port of tribox3.c)
//
// Original algorithm by Tomas Akenine-Möller.
// C++ / Eigen wrapper (replaces tribox3.c).
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
// Algorithm portions Copyright 2020 Tomas Akenine-Möller (MIT license).
//
////////////////////////////////////////////////////////////////////

#ifndef _TRIBOX3_HXX
#define _TRIBOX3_HXX 1

#include <array>
#include <cmath>

#include "myEigen.hxx"

// Separating-axis AABB vs triangle overlap (tribox3.c).
class TriBoxOverlap {
 public:
  template <typename Scalar>
  static bool overlap(const Eigen::Matrix<Scalar, 3, 1>& box_center,
                      const Eigen::Matrix<Scalar, 3, 1>& box_half_size,
                      const Eigen::Matrix<Scalar, 3, 1>& tri_v0,
                      const Eigen::Matrix<Scalar, 3, 1>& tri_v1,
                      const Eigen::Matrix<Scalar, 3, 1>& tri_v2) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

    const Vec3 v0 = tri_v0 - box_center;
    const Vec3 v1 = tri_v1 - box_center;
    const Vec3 v2 = tri_v2 - box_center;

    const Vec3 e0 = v1 - v0;
    const Vec3 e1 = v2 - v1;
    const Vec3 e2 = v0 - v2;

    if (!axisTests(e0, v0, v1, v2, box_half_size)) return false;
    if (!axisTests(e1, v0, v1, v2, box_half_size)) return false;
    if (!axisTests(e2, v0, v1, v2, box_half_size, /*edge_index=*/2)) return false;

    if (!aabbSlabTest(v0, v1, v2, box_half_size)) return false;

    const Vec3 normal = e0.cross(e1);
    return planeBoxOverlap(normal, v0, box_half_size);
  }

  static bool overlap(const Eigen::Vector3f& box_center,
                      const Eigen::Vector3f& box_half_size,
                      const Eigen::Vector3f& tri_v0,
                      const Eigen::Vector3f& tri_v1,
                      const Eigen::Vector3f& tri_v2) {
    return overlap<float>(box_center, box_half_size, tri_v0, tri_v1, tri_v2);
  }

  static bool overlap(const Eigen::Vector3d& box_center,
                      const Eigen::Vector3d& box_half_size,
                      const Eigen::Vector3d& tri_v0,
                      const Eigen::Vector3d& tri_v1,
                      const Eigen::Vector3d& tri_v2) {
    return overlap<double>(box_center, box_half_size, tri_v0, tri_v1, tri_v2);
  }

  static bool overlap(const Eigen::Vector3d& box_center,
                      const Eigen::Vector3d& box_half_size,
                      const std::array<Eigen::Vector3d, 3>& triangle) {
    return overlap(box_center, box_half_size, triangle[0], triangle[1], triangle[2]);
  }

 private:
  template <typename Scalar>
  static bool planeBoxOverlap(const Eigen::Matrix<Scalar, 3, 1>& normal,
                              const Eigen::Matrix<Scalar, 3, 1>& vert,
                              const Eigen::Matrix<Scalar, 3, 1>& maxbox) {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    Vec3 vmin;
    Vec3 vmax;
    for (int q = 0; q < 3; ++q) {
      const Scalar v = vert[q];
      if (normal[q] > Scalar(0)) {
        vmin[q] = -maxbox[q] - v;
        vmax[q] = maxbox[q] - v;
      } else {
        vmin[q] = maxbox[q] - v;
        vmax[q] = -maxbox[q] - v;
      }
    }
    if (normal.dot(vmin) > Scalar(0)) return false;
    if (normal.dot(vmax) >= Scalar(0)) return true;
    return false;
  }

  template <typename Scalar>
  static void findMinMax(Scalar x0, Scalar x1, Scalar x2, Scalar& min_out,
                         Scalar& max_out) {
    min_out = max_out = x0;
    if (x1 < min_out) min_out = x1;
    if (x1 > max_out) max_out = x1;
    if (x2 < min_out) min_out = x2;
    if (x2 > max_out) max_out = x2;
  }

  template <typename Scalar>
  static bool aabbSlabTest(const Eigen::Matrix<Scalar, 3, 1>& v0,
                           const Eigen::Matrix<Scalar, 3, 1>& v1,
                           const Eigen::Matrix<Scalar, 3, 1>& v2,
                           const Eigen::Matrix<Scalar, 3, 1>& box_half_size) {
    Scalar min_val, max_val;
    findMinMax(v0.x(), v1.x(), v2.x(), min_val, max_val);
    if (min_val > box_half_size.x() || max_val < -box_half_size.x()) return false;

    findMinMax(v0.y(), v1.y(), v2.y(), min_val, max_val);
    if (min_val > box_half_size.y() || max_val < -box_half_size.y()) return false;

    findMinMax(v0.z(), v1.z(), v2.z(), min_val, max_val);
    if (min_val > box_half_size.z() || max_val < -box_half_size.z()) return false;
    return true;
  }

  template <typename Scalar>
  static bool axisTestX01(Scalar a, Scalar b, Scalar fa, Scalar fb,
                          const Eigen::Matrix<Scalar, 3, 1>& v0,
                          const Eigen::Matrix<Scalar, 3, 1>& v2,
                          const Eigen::Matrix<Scalar, 3, 1>& box_half_size) {
    const Scalar p0 = a * v0.y() - b * v0.z();
    const Scalar p2 = a * v2.y() - b * v2.z();
    const Scalar min_val = std::min(p0, p2);
    const Scalar max_val = std::max(p0, p2);
    const Scalar rad = fa * box_half_size.y() + fb * box_half_size.z();
    return !(min_val > rad || max_val < -rad);
  }

  template <typename Scalar>
  static bool axisTestX2(Scalar a, Scalar b, Scalar fa, Scalar fb,
                         const Eigen::Matrix<Scalar, 3, 1>& v0,
                         const Eigen::Matrix<Scalar, 3, 1>& v1,
                         const Eigen::Matrix<Scalar, 3, 1>& box_half_size) {
    const Scalar p0 = a * v0.y() - b * v0.z();
    const Scalar p1 = a * v1.y() - b * v1.z();
    const Scalar min_val = std::min(p0, p1);
    const Scalar max_val = std::max(p0, p1);
    const Scalar rad = fa * box_half_size.y() + fb * box_half_size.z();
    return !(min_val > rad || max_val < -rad);
  }

  template <typename Scalar>
  static bool axisTestY02(Scalar a, Scalar b, Scalar fa, Scalar fb,
                          const Eigen::Matrix<Scalar, 3, 1>& v0,
                          const Eigen::Matrix<Scalar, 3, 1>& v2,
                          const Eigen::Matrix<Scalar, 3, 1>& box_half_size) {
    const Scalar p0 = -a * v0.x() + b * v0.z();
    const Scalar p2 = -a * v2.x() + b * v2.z();
    const Scalar min_val = std::min(p0, p2);
    const Scalar max_val = std::max(p0, p2);
    const Scalar rad = fa * box_half_size.x() + fb * box_half_size.z();
    return !(min_val > rad || max_val < -rad);
  }

  template <typename Scalar>
  static bool axisTestY1(Scalar a, Scalar b, Scalar fa, Scalar fb,
                         const Eigen::Matrix<Scalar, 3, 1>& v0,
                         const Eigen::Matrix<Scalar, 3, 1>& v1,
                         const Eigen::Matrix<Scalar, 3, 1>& box_half_size) {
    const Scalar p0 = -a * v0.x() + b * v0.z();
    const Scalar p1 = -a * v1.x() + b * v1.z();
    const Scalar min_val = std::min(p0, p1);
    const Scalar max_val = std::max(p0, p1);
    const Scalar rad = fa * box_half_size.x() + fb * box_half_size.z();
    return !(min_val > rad || max_val < -rad);
  }

  template <typename Scalar>
  static bool axisTestZ12(Scalar a, Scalar b, Scalar fa, Scalar fb,
                          const Eigen::Matrix<Scalar, 3, 1>& v1,
                          const Eigen::Matrix<Scalar, 3, 1>& v2,
                          const Eigen::Matrix<Scalar, 3, 1>& box_half_size) {
    const Scalar p1 = a * v1.x() - b * v1.y();
    const Scalar p2 = a * v2.x() - b * v2.y();
    const Scalar min_val = std::min(p1, p2);
    const Scalar max_val = std::max(p1, p2);
    const Scalar rad = fa * box_half_size.x() + fb * box_half_size.y();
    return !(min_val > rad || max_val < -rad);
  }

  template <typename Scalar>
  static bool axisTestZ0(Scalar a, Scalar b, Scalar fa, Scalar fb,
                         const Eigen::Matrix<Scalar, 3, 1>& v0,
                         const Eigen::Matrix<Scalar, 3, 1>& v1,
                         const Eigen::Matrix<Scalar, 3, 1>& box_half_size) {
    const Scalar p0 = a * v0.x() - b * v0.y();
    const Scalar p1 = a * v1.x() - b * v1.y();
    const Scalar min_val = std::min(p0, p1);
    const Scalar max_val = std::max(p0, p1);
    const Scalar rad = fa * box_half_size.x() + fb * box_half_size.y();
    return !(min_val > rad || max_val < -rad);
  }

  template <typename Scalar>
  static bool axisTests(const Eigen::Matrix<Scalar, 3, 1>& edge,
                        const Eigen::Matrix<Scalar, 3, 1>& v0,
                        const Eigen::Matrix<Scalar, 3, 1>& v1,
                        const Eigen::Matrix<Scalar, 3, 1>& v2,
                        const Eigen::Matrix<Scalar, 3, 1>& box_half_size,
                        int edge_index = 0) {
    const Scalar fex = std::abs(edge.x());
    const Scalar fey = std::abs(edge.y());
    const Scalar fez = std::abs(edge.z());

    if (edge_index == 0) {
      if (!axisTestX01(edge.z(), edge.y(), fez, fey, v0, v2, box_half_size)) return false;
      if (!axisTestY02(edge.z(), edge.x(), fez, fex, v0, v2, box_half_size)) return false;
      if (!axisTestZ12(edge.y(), edge.x(), fey, fex, v1, v2, box_half_size)) return false;
    } else if (edge_index == 1) {
      if (!axisTestX01(edge.z(), edge.y(), fez, fey, v0, v2, box_half_size)) return false;
      if (!axisTestY02(edge.z(), edge.x(), fez, fex, v0, v2, box_half_size)) return false;
      if (!axisTestZ0(edge.y(), edge.x(), fey, fex, v0, v1, box_half_size)) return false;
    } else {
      if (!axisTestX2(edge.z(), edge.y(), fez, fey, v0, v1, box_half_size)) return false;
      if (!axisTestY1(edge.z(), edge.x(), fez, fex, v0, v1, box_half_size)) return false;
      if (!axisTestZ12(edge.y(), edge.x(), fey, fex, v1, v2, box_half_size)) return false;
    }
    return true;
  }
};

#endif  // _TRIBOX3_HXX
