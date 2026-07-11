////////////////////////////////////////////////////////////////////
//
// Symmetric Dirichlet energy for UV parameterization (Eigen charts)
//
// Extracted from optcuts/OptCutsL.hxx for shared use.
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _SYMDIRICHLETENERGY_HXX
#define _SYMDIRICHLETENERGY_HXX 1

#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "myEigen.hxx"

namespace symdirichlet {

inline constexpr double kEps = 1.0e-12;

inline double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}

inline double triangle3DArea(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                             const Eigen::Vector3d& p2) {
  return 0.5 * (p1 - p0).cross(p2 - p0).norm();
}

inline double meshSurfaceArea(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
  double area = 0.0;
  for (int fi = 0; fi < F.rows(); ++fi) {
    area += triangle3DArea(V.row(F(fi, 0)), V.row(F(fi, 1)), V.row(F(fi, 2)));
  }
  return area;
}

inline double triangleEnergy(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                               const Eigen::Vector3d& p2, const Eigen::Vector2d& u0,
                               const Eigen::Vector2d& u1, const Eigen::Vector2d& u2,
                               double weight, bool require_positive_area,
                               Eigen::Vector2d* g0 = nullptr,
                               Eigen::Vector2d* g1 = nullptr,
                               Eigen::Vector2d* g2 = nullptr) {
  const Eigen::Vector3d p2m1 = p1 - p0;
  const Eigen::Vector3d p3m1 = p2 - p0;
  const double area = 0.5 * p2m1.cross(p3m1).norm();
  const double area_sq = area * area;
  if (area <= kEps) return std::numeric_limits<double>::infinity();

  const double e0_sq_len = p2m1.squaredNorm();
  const double e1_sq_len = p3m1.squaredNorm();
  const double e0_dot_e1 = p2m1.dot(p3m1);

  const Eigen::Vector2d u2m1 = u1 - u0;
  const Eigen::Vector2d u3m1 = u2 - u0;
  const double area_uv = 0.5 * cross2(u2m1, u3m1);

  if ((require_positive_area && area_uv <= kEps) || std::fabs(area_uv) <= kEps) {
    return std::numeric_limits<double>::infinity();
  }

  const double left = 1.0 + area_sq / (area_uv * area_uv);
  const double right = (u3m1.squaredNorm() * e0_sq_len + u2m1.squaredNorm() * e1_sq_len) /
                           (4.0 * area_sq) -
                       u3m1.dot(u2m1) * e0_dot_e1 / (2.0 * area_sq);
  const double energy = weight * left * right;

  if (g0 || g1 || g2) {
    const double area_ratio = area_sq / (area_uv * area_uv * area_uv);
    const Eigen::Vector2d edge_oppo1 = u2 - u1;
    const Eigen::Vector2d d_left1(area_ratio * edge_oppo1.y(), -area_ratio * edge_oppo1.x());
    const Eigen::Vector2d d_right1 =
        ((e0_dot_e1 - e0_sq_len) * u3m1 + (e0_dot_e1 - e1_sq_len) * u2m1) / (2.0 * area_sq);
    const Eigen::Vector2d grad0 = weight * (d_left1 * right + d_right1 * left);

    const Eigen::Vector2d edge_oppo2 = u0 - u2;
    const Eigen::Vector2d d_left2(area_ratio * edge_oppo2.y(), -area_ratio * edge_oppo2.x());
    const Eigen::Vector2d d_right2 =
        (e1_sq_len * u2m1 - e0_dot_e1 * u3m1) / (2.0 * area_sq);
    const Eigen::Vector2d grad1 = weight * (d_left2 * right + d_right2 * left);

    const Eigen::Vector2d edge_oppo3 = u1 - u0;
    const Eigen::Vector2d d_left3(area_ratio * edge_oppo3.y(), -area_ratio * edge_oppo3.x());
    const Eigen::Vector2d d_right3 =
        (e0_sq_len * u3m1 - e0_dot_e1 * u2m1) / (2.0 * area_sq);
    const Eigen::Vector2d grad2 = weight * (d_left3 * right + d_right3 * left);

    if (g0) *g0 = grad0;
    if (g1) *g1 = grad1;
    if (g2) *g2 = grad2;
  }

  return energy;
}

inline double meshEnergyAndGradient(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                    const Eigen::MatrixXd& UV,
                                    std::vector<Eigen::Vector2d>* gradient,
                                    bool require_positive_area,
                                    double surface_area = 0.0) {
  if (surface_area <= kEps) surface_area = meshSurfaceArea(V, F);
  surface_area = std::max(surface_area, kEps);

  if (gradient) gradient->assign(UV.rows(), Eigen::Vector2d::Zero());

  double energy = 0.0;
  for (int fi = 0; fi < F.rows(); ++fi) {
    const int i0 = F(fi, 0);
    const int i1 = F(fi, 1);
    const int i2 = F(fi, 2);
    const double w =
        triangle3DArea(V.row(i0), V.row(i1), V.row(i2)) / surface_area;

    Eigen::Vector2d g0, g1, g2;
    const double e = triangleEnergy(
        V.row(i0), V.row(i1), V.row(i2), UV.row(i0), UV.row(i1), UV.row(i2), w,
        require_positive_area, gradient ? &g0 : nullptr, gradient ? &g1 : nullptr,
        gradient ? &g2 : nullptr);
    if (!std::isfinite(e)) return e;
    energy += e;
    if (gradient) {
      (*gradient)[i0] += g0;
      (*gradient)[i1] += g1;
      (*gradient)[i2] += g2;
    }
  }
  return energy;
}

inline std::set<int> boundaryVertexIndices(const Eigen::MatrixXi& F) {
  std::map<std::pair<int, int>, int> edge_count;
  for (int fi = 0; fi < F.rows(); ++fi) {
    for (int e = 0; e < 3; ++e) {
      int a = F(fi, e);
      int b = F(fi, (e + 1) % 3);
      if (a > b) std::swap(a, b);
      ++edge_count[{a, b}];
    }
  }
  std::set<int> boundary;
  for (const auto& kv : edge_count) {
    if (kv.second == 1) {
      boundary.insert(kv.first.first);
      boundary.insert(kv.first.second);
    }
  }
  return boundary;
}

}  // namespace symdirichlet

#endif  // _SYMDIRICHLETENERGY_HXX
