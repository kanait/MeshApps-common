////////////////////////////////////////////////////////////////////
//
// Lightweight UV air-mesh scaffold (Eigen, OptCuts-compatible)
//
// Builds an outward offset ring of air triangles around UV boundary
// loops so Symmetric Dirichlet on the air mesh can discourage
// boundary collapse / interior folding during free-boundary UV
// optimization (SCAF-like idea without Triangle CDT).
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _UVSCAFFOLD_HXX
#define _UVSCAFFOLD_HXX 1

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif

#include "SymDirichletEnergy.hxx"
#include "TriangleTriangulate.hxx"
#include "myEigen.hxx"

namespace uvscaffold {

inline constexpr double kEps = 1.0e-12;

// In-process Triangle CDT (q: min angle ~20°, Y: no Steiner on PSLG segments).
// Relies on triangle.cpp initializing vertex2tri to NULL after poolalloc —
// previously uninitialized links caused SIGSEGV in formskeleton after UV-AT's
// heap had reused dirty pages.
inline bool triangulateIsolated(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E,
                                const Eigen::MatrixXd& H, Eigen::MatrixXd& V_out,
                                Eigen::MatrixXi& F_out) {
  V_out.resize(0, 2);
  F_out.resize(0, 3);
  if (V.rows() < 3 || E.rows() < 3) return false;
  triangle_mesh::triangulate(V, E, H, "qYQ", V_out, F_out);
  return V_out.rows() >= 3 && F_out.rows() > 0;
}

enum class Mode { Lightweight, Triangle };

inline const char* modeName(Mode m) {
  return m == Mode::Triangle ? "triangle" : "lightweight";
}

struct AirVertex {
  bool fixed = false;
  int uv_id = -1;  // index into chart UV matrix X (when !fixed)
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
};

struct AirTriangle {
  std::array<int, 3> v = {{-1, -1, -1}};
  std::array<Eigen::Vector2d, 3> rest;
};

struct Scaffold {
  std::vector<AirVertex> vertices;
  std::vector<AirTriangle> triangles;
};

inline Eigen::Vector2d airVertexPosition(const AirVertex& v, const Eigen::MatrixXd& X) {
  if (v.fixed) return v.position;
  if (v.uv_id < 0 || v.uv_id >= X.rows()) return Eigen::Vector2d::Zero();
  return X.row(v.uv_id).transpose();
}

// Lightweight offset-ring scaffold (same construction as OptCutsL::buildLightweightScaffold).
inline Scaffold buildLightweight(const Eigen::MatrixXd& X,
                                 const std::vector<std::vector<int>>& boundary_loops,
                                 double avg_edge_length) {
  Scaffold scaffold;
  avg_edge_length = std::max(avg_edge_length, kEps);
  std::map<int, int> uv_to_air;

  auto add_uv_vertex = [&](int uvid) {
    auto inserted = uv_to_air.emplace(uvid, static_cast<int>(scaffold.vertices.size()));
    if (inserted.second) {
      AirVertex av;
      av.fixed = false;
      av.uv_id = uvid;
      scaffold.vertices.push_back(av);
    }
    return inserted.first->second;
  };

  auto add_fixed_vertex = [&](const Eigen::Vector2d& p) {
    AirVertex av;
    av.fixed = true;
    av.position = p;
    scaffold.vertices.push_back(av);
    return static_cast<int>(scaffold.vertices.size() - 1);
  };

  auto add_oriented_triangle = [&](int a, int b, int c) {
    AirTriangle tri;
    tri.v = {{a, b, c}};
    for (int i = 0; i < 3; ++i) {
      tri.rest[i] = airVertexPosition(scaffold.vertices[static_cast<size_t>(tri.v[i])], X);
    }
    const double area =
        0.5 * symdirichlet::cross2(tri.rest[1] - tri.rest[0], tri.rest[2] - tri.rest[0]);
    if (std::fabs(area) <= kEps) return;
    if (area < 0.0) {
      std::swap(tri.v[1], tri.v[2]);
      std::swap(tri.rest[1], tri.rest[2]);
    }
    scaffold.triangles.push_back(tri);
  };

  for (const auto& loop : boundary_loops) {
    const int n = static_cast<int>(loop.size());
    if (n < 3) continue;

    std::vector<Eigen::Vector2d> pos(static_cast<size_t>(n));
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double loop_len = 0.0;
    for (int i = 0; i < n; ++i) {
      const int vid = loop[static_cast<size_t>(i)];
      if (vid < 0 || vid >= X.rows()) {
        pos.clear();
        break;
      }
      pos[static_cast<size_t>(i)] = X.row(vid).transpose();
      center += pos[static_cast<size_t>(i)];
      loop_len += (X.row(loop[static_cast<size_t>((i + 1) % n)]) - X.row(vid)).norm();
    }
    if (static_cast<int>(pos.size()) != n || loop_len <= kEps) continue;
    center /= static_cast<double>(n);

    double radius = 0.0;
    for (int i = 0; i < n; ++i) {
      radius += (pos[static_cast<size_t>(i)] - center).norm();
    }
    radius = std::max(radius / static_cast<double>(n), avg_edge_length);
    const double offset =
        std::max(0.35 * radius, std::max(avg_edge_length, loop_len / static_cast<double>(n)));

    double signed_loop_area = 0.0;
    for (int i = 0; i < n; ++i) {
      signed_loop_area +=
          symdirichlet::cross2(pos[static_cast<size_t>(i)], pos[static_cast<size_t>((i + 1) % n)]);
    }

    auto outward_normal = [&](const Eigen::Vector2d& edge) -> Eigen::Vector2d {
      if (edge.norm() <= kEps || std::fabs(signed_loop_area) <= kEps) {
        return Eigen::Vector2d::Zero();
      }
      Eigen::Vector2d dir;
      if (signed_loop_area > 0.0) {
        dir = Eigen::Vector2d(edge.y(), -edge.x());
      } else {
        dir = Eigen::Vector2d(-edge.y(), edge.x());
      }
      const double len = dir.norm();
      if (len <= kEps) return Eigen::Vector2d::Zero();
      return dir / len;
    };

    std::vector<int> inner(static_cast<size_t>(n)), outer(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const int prev = (i + n - 1) % n;
      const int next = (i + 1) % n;
      const Eigen::Vector2d p = pos[static_cast<size_t>(i)];
      Eigen::Vector2d dir =
          outward_normal(pos[static_cast<size_t>(i)] - pos[static_cast<size_t>(prev)]) +
          outward_normal(pos[static_cast<size_t>(next)] - pos[static_cast<size_t>(i)]);
      if (dir.norm() <= kEps) dir = p - center;
      if (dir.norm() <= kEps) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
        dir = Eigen::Vector2d(std::cos(angle), std::sin(angle));
      } else {
        dir.normalize();
      }
      inner[static_cast<size_t>(i)] = add_uv_vertex(loop[static_cast<size_t>(i)]);
      outer[static_cast<size_t>(i)] = add_fixed_vertex(p + offset * dir);
    }

    for (int i = 0; i < n; ++i) {
      const int j = (i + 1) % n;
      add_oriented_triangle(inner[static_cast<size_t>(i)], inner[static_cast<size_t>(j)],
                            outer[static_cast<size_t>(j)]);
      add_oriented_triangle(inner[static_cast<size_t>(i)], outer[static_cast<size_t>(j)],
                            outer[static_cast<size_t>(i)]);
    }
  }

  return scaffold;
}

// SCAF-style outer fill via Triangle CDT (OptCutsL::buildTriangleScaffold).
// Movable verts = boundary UV; all Steiner / bbox verts are fixed.
// Falls back to Lightweight if Triangle fails.
inline Scaffold buildTriangle(const Eigen::MatrixXd& X, const Eigen::MatrixXi& F,
                              const std::vector<std::vector<int>>& boundary_loops,
                              double avg_edge_length) {
  Scaffold scaffold;
  avg_edge_length = std::max(avg_edge_length, kEps);
  if (boundary_loops.empty() || X.rows() < 3 || F.rows() < 1) {
    return buildLightweight(X, boundary_loops, avg_edge_length);
  }

  Eigen::MatrixXd Vpslg;
  Eigen::MatrixXi E;
  std::vector<int> bnd_uv_ids;

  auto append_loop = [&](const std::vector<int>& loop) {
    const int n = static_cast<int>(loop.size());
    if (n < 3) return;

    std::vector<int> filtered;
    filtered.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const int uvid = loop[static_cast<size_t>(i)];
      if (uvid < 0 || uvid >= X.rows()) return;
      if (!filtered.empty()) {
        const int prev = filtered.back();
        if ((X.row(uvid) - X.row(prev)).norm() <= kEps) continue;
      }
      filtered.push_back(uvid);
    }
    if (filtered.size() >= 2 &&
        (X.row(filtered.front()) - X.row(filtered.back())).norm() <= kEps) {
      filtered.pop_back();
    }
    const int nf = static_cast<int>(filtered.size());
    if (nf < 3) return;

    double loop_len = 0.0;
    for (int i = 0; i < nf; ++i) {
      loop_len +=
          (X.row(filtered[static_cast<size_t>(i)]) -
           X.row(filtered[static_cast<size_t>((i + 1) % nf)]))
              .norm();
    }
    if (loop_len <= kEps) return;

    const int base = static_cast<int>(Vpslg.rows());
    Vpslg.conservativeResize(base + nf, 2);
    E.conservativeResize(E.rows() + nf, 2);
    bnd_uv_ids.reserve(bnd_uv_ids.size() + static_cast<size_t>(nf));
    for (int i = 0; i < nf; ++i) {
      const int uvid = filtered[static_cast<size_t>(i)];
      Vpslg.row(base + i) = X.row(uvid);
      bnd_uv_ids.push_back(uvid);
      E.row(E.rows() - nf + i) << base + i, base + ((i + 1) % nf);
    }
  };

  for (const auto& loop : boundary_loops) {
    append_loop(loop);
  }

  // Prefer a single outer loop (longest) for a disk-like chart. Extra tiny
  // loops after AT cuts can make Triangle PSLG ill-posed / crash.
  if (boundary_loops.size() > 1) {
    size_t best = 0;
    for (size_t i = 1; i < boundary_loops.size(); ++i) {
      if (boundary_loops[i].size() > boundary_loops[best].size()) best = i;
    }
    Vpslg.resize(0, 2);
    E.resize(0, 2);
    bnd_uv_ids.clear();
    append_loop(boundary_loops[best]);
  }

  const int n_bnd = static_cast<int>(Vpslg.rows());
  if (n_bnd < 3) return buildLightweight(X, boundary_loops, avg_edge_length);

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < X.rows(); ++i) {
    min_x = std::min(min_x, X(i, 0));
    max_x = std::max(max_x, X(i, 0));
    min_y = std::min(min_y, X(i, 1));
    max_y = std::max(max_y, X(i, 1));
  }

  const double scale_factor = 2.0;
  const int band_amt = 1;
  // Prefer UV-scale edge length so bbox resolution tracks the chart, not 3D size.
  double uv_edge = 0.0;
  int uv_edge_count = 0;
  for (int i = 0; i < n_bnd; ++i) {
    const Eigen::RowVector2d a = Vpslg.row(i);
    const Eigen::RowVector2d b = Vpslg.row((i + 1) % n_bnd);
    const double len = (a - b).norm();
    if (len > kEps) {
      uv_edge += len;
      ++uv_edge_count;
    }
  }
  const double edge_ref =
      (uv_edge_count > 0) ? (uv_edge / static_cast<double>(uv_edge_count))
                          : std::max(avg_edge_length, kEps);
  // Finer exterior mesh than OptCuts defaults — better interior-overlap control.
  const double seg_len = std::max(edge_ref * std::pow(scale_factor, static_cast<double>(band_amt)),
                                  0.5 * edge_ref);
  const double margin = std::max(3.0 * edge_ref, 2.0 * seg_len);
  min_x -= margin;
  max_x += margin;
  min_y -= margin;
  max_y += margin;

  int seg_amt_x =
      std::max(1, static_cast<int>(std::ceil((max_x - min_x) / std::max(seg_len, kEps))));
  int seg_amt_y =
      std::max(1, static_cast<int>(std::ceil((max_y - min_y) / std::max(seg_len, kEps))));
  constexpr int kMaxBBoxSegPerSide = 128;
  seg_amt_x = std::min(seg_amt_x, kMaxBBoxSegPerSide);
  seg_amt_y = std::min(seg_amt_y, kMaxBBoxSegPerSide);
  const int bbox_amt = (seg_amt_x + seg_amt_y) * 2;
  const int bbox_start = n_bnd;

  E.conservativeResize(E.rows() + bbox_amt, 2);
  for (int seg_i = bbox_start; seg_i + 1 < E.rows(); ++seg_i) {
    E.row(seg_i) << seg_i, seg_i + 1;
  }
  E.row(E.rows() - 1) << E.rows() - 1, bbox_start;

  const double step_x = (max_x - min_x) / static_cast<double>(seg_amt_x);
  const double step_y = (max_y - min_y) / static_cast<double>(seg_amt_y);
  Vpslg.conservativeResize(bbox_start + bbox_amt, 2);
  for (int seg_i = 0; seg_i < seg_amt_x; ++seg_i) {
    Vpslg.row(bbox_start + seg_i) << min_x + seg_i * step_x, min_y;
  }
  for (int seg_i = 0; seg_i < seg_amt_y; ++seg_i) {
    Vpslg.row(bbox_start + seg_amt_x + seg_i) << max_x, min_y + seg_i * step_y;
  }
  for (int seg_i = 0; seg_i < seg_amt_x; ++seg_i) {
    Vpslg.row(bbox_start + seg_amt_x + seg_amt_y + seg_i)
        << max_x - seg_i * step_x, max_y;
  }
  for (int seg_i = 0; seg_i < seg_amt_y; ++seg_i) {
    Vpslg.row(bbox_start + 2 * seg_amt_x + seg_amt_y + seg_i)
        << min_x, max_y - seg_i * step_y;
  }

  Eigen::Vector2d hole = Eigen::Vector2d::Zero();
  int hole_count = 0;
  for (int fi = 0; fi < F.rows(); ++fi) {
    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    bool ok = true;
    for (int k = 0; k < 3; ++k) {
      const int vid = F(fi, k);
      if (vid < 0 || vid >= X.rows()) {
        ok = false;
        break;
      }
      c += X.row(vid).transpose();
    }
    if (!ok) continue;
    hole += c / 3.0;
    ++hole_count;
  }
  if (hole_count == 0) return buildLightweight(X, boundary_loops, avg_edge_length);
  hole /= static_cast<double>(hole_count);

  Eigen::MatrixXd H(1, 2);
  H.row(0) = hole.transpose();

  Eigen::MatrixXd V_out;
  Eigen::MatrixXi F_out;
  if (!triangulateIsolated(Vpslg, E, H, V_out, F_out)) {
    return buildLightweight(X, boundary_loops, avg_edge_length);
  }

  auto add_oriented_triangle = [&](int a, int b, int c) {
    AirTriangle tri;
    tri.v = {{a, b, c}};
    for (int i = 0; i < 3; ++i) {
      tri.rest[i] = airVertexPosition(scaffold.vertices[static_cast<size_t>(tri.v[i])], X);
    }
    const double area =
        0.5 * symdirichlet::cross2(tri.rest[1] - tri.rest[0], tri.rest[2] - tri.rest[0]);
    if (std::fabs(area) <= kEps) return;
    if (area < 0.0) {
      std::swap(tri.v[1], tri.v[2]);
      std::swap(tri.rest[1], tri.rest[2]);
    }
    scaffold.triangles.push_back(tri);
  };

  scaffold.vertices.reserve(static_cast<size_t>(V_out.rows()));
  for (int vi = 0; vi < V_out.rows(); ++vi) {
    AirVertex av;
    if (vi < n_bnd) {
      av.fixed = false;
      av.uv_id = bnd_uv_ids[static_cast<size_t>(vi)];
    } else {
      av.fixed = true;
      av.uv_id = -1;
      av.position = V_out.row(vi).transpose();
    }
    scaffold.vertices.push_back(av);
  }

  for (int fi = 0; fi < F_out.rows(); ++fi) {
    add_oriented_triangle(F_out(fi, 0), F_out(fi, 1), F_out(fi, 2));
  }

  if (scaffold.triangles.empty()) {
    return buildLightweight(X, boundary_loops, avg_edge_length);
  }
  return scaffold;
}

inline Scaffold build(Mode mode, const Eigen::MatrixXd& X, const Eigen::MatrixXi& F,
                      const std::vector<std::vector<int>>& boundary_loops,
                      double avg_edge_length) {
  if (mode == Mode::Triangle) {
    return buildTriangle(X, F, boundary_loops, avg_edge_length);
  }
  return buildLightweight(X, boundary_loops, avg_edge_length);
}

inline double energyAndGradient(const Scaffold& scaffold, const Eigen::MatrixXd& X,
                                double external_weight, Eigen::VectorXd* grad_xy,
                                bool require_positive_area) {
  if (scaffold.triangles.empty() || external_weight <= 0.0) return 0.0;
  const int n = static_cast<int>(X.rows());
  if (grad_xy) grad_xy->setZero(2 * n);

  double energy = 0.0;
  const double tri_w =
      external_weight / static_cast<double>(scaffold.triangles.size());

  for (const auto& tri : scaffold.triangles) {
    const Eigen::Vector3d P1(tri.rest[0].x(), tri.rest[0].y(), 0.0);
    const Eigen::Vector3d P2(tri.rest[1].x(), tri.rest[1].y(), 0.0);
    const Eigen::Vector3d P3(tri.rest[2].x(), tri.rest[2].y(), 0.0);
    const Eigen::Vector2d U1 =
        airVertexPosition(scaffold.vertices[static_cast<size_t>(tri.v[0])], X);
    const Eigen::Vector2d U2 =
        airVertexPosition(scaffold.vertices[static_cast<size_t>(tri.v[1])], X);
    const Eigen::Vector2d U3 =
        airVertexPosition(scaffold.vertices[static_cast<size_t>(tri.v[2])], X);

    Eigen::Vector2d g0, g1, g2;
    const double e = symdirichlet::triangleEnergy(
        P1, P2, P3, U1, U2, U3, tri_w, require_positive_area,
        grad_xy ? &g0 : nullptr, grad_xy ? &g1 : nullptr, grad_xy ? &g2 : nullptr);
    if (!std::isfinite(e)) return std::numeric_limits<double>::infinity();
    energy += e;

    if (grad_xy) {
      const AirVertex* avs[3] = {&scaffold.vertices[static_cast<size_t>(tri.v[0])],
                                 &scaffold.vertices[static_cast<size_t>(tri.v[1])],
                                 &scaffold.vertices[static_cast<size_t>(tri.v[2])]};
      const Eigen::Vector2d* gs[3] = {&g0, &g1, &g2};
      for (int k = 0; k < 3; ++k) {
        if (avs[k]->fixed || avs[k]->uv_id < 0 || avs[k]->uv_id >= n) continue;
        (*grad_xy)(avs[k]->uv_id) += (*gs[k]).x();
        (*grad_xy)(n + avs[k]->uv_id) += (*gs[k]).y();
      }
    }
  }
  return energy;
}

inline double energy(const Scaffold& scaffold, const Eigen::MatrixXd& X,
                     double external_weight, bool require_positive_area = true) {
  return energyAndGradient(scaffold, X, external_weight, nullptr, require_positive_area);
}

// Max flip-free step along direction D for air triangles (fixed verts have zero velocity).
inline double maxStepFromSingularities(const Scaffold& scaffold, const Eigen::MatrixXd& X,
                                       const Eigen::MatrixXd& D) {
  double max_step = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd uv(3, 2);
  Eigen::MatrixXd d(3, 2);
  Eigen::MatrixXi F(1, 3);
  F << 0, 1, 2;

  for (const auto& tri : scaffold.triangles) {
    for (int k = 0; k < 3; ++k) {
      const AirVertex& av = scaffold.vertices[static_cast<size_t>(tri.v[k])];
      uv.row(k) = airVertexPosition(av, X).transpose();
      if (av.fixed || av.uv_id < 0 || av.uv_id >= D.rows()) {
        d.row(k).setZero();
      } else {
        d.row(k) = D.row(av.uv_id);
      }
    }
    // Reuse the same quadratic root as mesh faces (via local 3-vert system).
    const double U11 = uv(0, 0), U12 = uv(0, 1);
    const double U21 = uv(1, 0), U22 = uv(1, 1);
    const double U31 = uv(2, 0), U32 = uv(2, 1);
    const double V11 = d(0, 0), V12 = d(0, 1);
    const double V21 = d(1, 0), V22 = d(1, 1);
    const double V31 = d(2, 0), V32 = d(2, 1);
    const double a =
        V11 * V22 - V12 * V21 - V11 * V32 + V12 * V31 + V21 * V32 - V22 * V31;
    const double b =
        U11 * V22 - U12 * V21 - U21 * V12 + U22 * V11 - U11 * V32 + U12 * V31 + U31 * V12 -
        U32 * V11 + U21 * V32 - U22 * V31 - U31 * V22 + U32 * V21;
    const double c =
        U11 * U22 - U12 * U21 - U11 * U32 + U12 * U31 + U21 * U32 - U22 * U31;

    auto smallestPositiveQuadraticRoot = [](double aa, double bb, double cc) {
      if (std::abs(aa) > 1.0e-10) {
        const double delta_in = bb * bb - 4.0 * aa * cc;
        if (delta_in <= 0.0) return std::numeric_limits<double>::infinity();
        const double delta = std::sqrt(delta_in);
        double t1, t2;
        if (bb >= 0.0) {
          const double bd = -bb - delta;
          t1 = 2.0 * cc / bd;
          t2 = bd / (2.0 * aa);
        } else {
          const double bd = -bb + delta;
          t1 = bd / (2.0 * aa);
          t2 = (2.0 * cc) / bd;
        }
        if (aa < 0.0) std::swap(t1, t2);
        if (t1 > 0.0) return t2 > 0.0 ? t2 : t1;
        return std::numeric_limits<double>::infinity();
      }
      if (std::abs(bb) < 1.0e-30) return std::numeric_limits<double>::infinity();
      const double t = -cc / bb;
      return t > 0.0 ? t : std::numeric_limits<double>::infinity();
    };

    max_step = std::min(max_step, smallestPositiveQuadraticRoot(a, b, c));
  }
  return max_step;
}

}  // namespace uvscaffold

#endif  // _UVSCAFFOLD_HXX
