////////////////////////////////////////////////////////////////////
//
// Per-face UV distortion metrics for distortion colormaps (partuv / optcuts)
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _UVDISTORTION_HXX
#define _UVDISTORTION_HXX 1

#include <cmath>
#include <limits>

#include "SymDirichletEnergy.hxx"
#include "myEigen.hxx"

namespace uvdistortion {

inline constexpr double kEps = 1.0e-12;

// partuv face colormap: max(A_uv/A_3d, A_3d/A_uv)
inline double triangleAreaRatioStretch(const Eigen::Vector3d& p0,
                                       const Eigen::Vector3d& p1,
                                       const Eigen::Vector3d& p2,
                                       const Eigen::Vector2d& u0,
                                       const Eigen::Vector2d& u1,
                                       const Eigen::Vector2d& u2) {
  const double a3 = 0.5 * (p1 - p0).cross(p2 - p0).norm();
  const double a2 =
      0.5 * std::abs(symdirichlet::cross2(u1 - u0, u2 - u0));
  if (a3 <= kEps) return 1.0;
  const double ratio = a2 / a3;
  return ratio < 1.0 ? 1.0 / ratio : ratio;
}

// optcuts face colormap: unweighted symmetric Dirichlet energy per triangle
inline double triangleSymmetricDirichlet(const Eigen::Vector3d& p0,
                                         const Eigen::Vector3d& p1,
                                         const Eigen::Vector3d& p2,
                                         const Eigen::Vector2d& u0,
                                         const Eigen::Vector2d& u1,
                                         const Eigen::Vector2d& u2) {
  const double e =
      symdirichlet::triangleEnergy(p0, p1, p2, u0, u1, u2, 1.0, false);
  return std::isfinite(e) ? e : std::numeric_limits<double>::infinity();
}

enum class ColormapMetric { AreaRatioStretch, SymmetricDirichlet };

}  // namespace uvdistortion

#endif  // _UVDISTORTION_HXX
