////////////////////////////////////////////////////////////////////
//
// Shared mesh UV parameterization API (LSCM / ARAP / Symmetric Dirichlet)
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _MESHPARAM_HXX
#define _MESHPARAM_HXX 1

#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>

#include "ArapParam.hxx"
#include "LscmParam.hxx"
#include "MeshCut.hxx"
#include "MeshL.hxx"
#include "SymDirichletParam.hxx"
#include "myEigen.hxx"

namespace meshparam {

enum class Method { LSCM, ARAP, SymDirichlet };

inline bool parseMethod(const std::string& name, Method& out) {
  if (name == "lscm" || name == "LSCM") {
    out = Method::LSCM;
    return true;
  }
  if (name == "arap" || name == "ARAP") {
    out = Method::ARAP;
    return true;
  }
  if (name == "symdirichlet" || name == "symmetric-dirichlet" ||
      name == "sd" || name == "SymDirichlet") {
    out = Method::SymDirichlet;
    return true;
  }
  return false;
}

inline const char* methodName(Method method) {
  switch (method) {
    case Method::LSCM:
      return "lscm";
    case Method::ARAP:
      return "arap";
    case Method::SymDirichlet:
      return "symdirichlet";
  }
  return "lscm";
}

// Chart building/merging uses a fast method; Symmetric Dirichlet is applied only
// on the final per-chart UV (see partuv::refineChartsParameterization).
inline Method chartStructureMethod(Method user) {
  if (user == Method::SymDirichlet) return Method::ARAP;
  return user;
}

inline bool meshFromEigen(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                          std::shared_ptr<MeshL>& mesh) {
  mesh = std::make_shared<MeshL>();
  std::vector<std::shared_ptr<VertexL>> verts(V.rows());
  for (int i = 0; i < V.rows(); ++i) {
    Eigen::Vector3d p(V(i, 0), V(i, 1), V(i, 2));
    verts[i] = mesh->addVertex(p);
  }
  for (int fi = 0; fi < F.rows(); ++fi) {
    if (F(fi, 0) < 0 || F(fi, 1) < 0 || F(fi, 2) < 0 || F(fi, 0) >= V.rows() ||
        F(fi, 1) >= V.rows() || F(fi, 2) >= V.rows()) {
      return false;
    }
    mesh->addTriangle(verts[F(fi, 0)], verts[F(fi, 1)], verts[F(fi, 2)]);
  }
  const bool restore = MeshL::connectivityWarnings();
  MeshL::setConnectivityWarnings(false);
  mesh->createConnectivity(true);
  MeshL::setConnectivityWarnings(restore);
  return true;
}

inline bool uvFromMesh(const std::shared_ptr<MeshL>& mesh, const Eigen::MatrixXd& V,
                       Eigen::MatrixXd& UV) {
  UV.resize(V.rows(), 2);
  UV.setZero();
  std::map<std::shared_ptr<VertexL>, int> vid;
  int i = 0;
  for (const auto& vt : mesh->vertices()) vid[vt] = i++;
  for (const auto& fc : mesh->faces()) {
    for (const auto& he : fc->halfedges()) {
      const auto tc = he->texcoord();
      if (!tc) continue;
      auto it = vid.find(he->vertex());
      if (it == vid.end()) continue;
      UV.row(it->second) = tc->point().head<2>().transpose();
    }
  }
  return true;
}

inline bool applyMethod(const std::shared_ptr<MeshL>& mesh, Method method) {
  switch (method) {
    case Method::LSCM: {
      LscmParam param(mesh);
      param.setVerbose(false);
      param.setQuiet(true);
      return param.apply();
    }
    case Method::ARAP: {
      ArapParam param(mesh);
      param.setInitMode(ArapParam::InitMode::Tutte);
      param.setVerbose(false);
      param.setQuiet(true);
      return param.apply();
    }
    case Method::SymDirichlet: {
      SymDirichletParam param(mesh);
      param.setVerbose(false);
      param.setQuiet(true);
      return param.apply();
    }
  }
  return false;
}

inline bool parameterize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                         Eigen::MatrixXd& UV, Method method = Method::LSCM) {
  const int orig_n = static_cast<int>(V.rows());
  Eigen::MatrixXd Vw = V;
  Eigen::MatrixXi Fw = F;
  std::vector<int> vtx_orig;
  if (!meshcut::meshFarthestPointCut(Vw, Fw, &vtx_orig)) return false;

  std::shared_ptr<MeshL> mesh;
  if (!meshFromEigen(Vw, Fw, mesh)) return false;
  if (!applyMethod(mesh, method)) return false;

  Eigen::MatrixXd UVw;
  if (!uvFromMesh(mesh, Vw, UVw)) return false;
  if (static_cast<int>(Vw.rows()) == orig_n) {
    UV = UVw;
  } else {
    meshcut::collapseUvAlongCut(UVw, vtx_orig, orig_n, UV);
  }
  return true;
}

// Relax symmetric Dirichlet energy from an existing UV (e.g. ARAP). When
// pin_boundary is true, chart boundary vertices stay fixed (better edges).
inline bool relaxSymmetricDirichlet(Eigen::MatrixXd& UV, const Eigen::MatrixXd& V,
                                    const Eigen::MatrixXi& F,
                                    bool pin_boundary = true,
                                    int max_iterations = 60) {
  if (UV.rows() != V.rows() || F.rows() == 0) return false;

  double signed_area = 0.0;
  for (int fi = 0; fi < F.rows(); ++fi) {
    const int i0 = F(fi, 0);
    const int i1 = F(fi, 1);
    const int i2 = F(fi, 2);
    signed_area += symdirichlet::cross2(UV.row(i1) - UV.row(i0),
                                        UV.row(i2) - UV.row(i0));
  }
  if (signed_area < 0.0) UV.col(1) *= -1.0;

  std::set<int> fixed;
  if (pin_boundary) fixed = symdirichlet::boundaryVertexIndices(F);
  if (fixed.empty() && UV.rows() > 0) fixed.insert(0);

  const double surface_area = symdirichlet::meshSurfaceArea(V, F);
  double edge_sum = 0.0;
  int edge_count = 0;
  for (int f = 0; f < F.rows(); ++f) {
    for (int e = 0; e < 3; ++e) {
      const int a = F(f, e);
      const int b = F(f, (e + 1) % 3);
      edge_sum += (V.row(a) - V.row(b)).norm();
      ++edge_count;
    }
  }
  const double avg_edge_length = edge_count > 0 ? edge_sum / edge_count : 1.0;
  const double chart_scale =
      std::max(std::sqrt(std::max(surface_area, symdirichlet::kEps)), avg_edge_length);

  double energy = symdirichlet::meshEnergyAndGradient(V, F, UV, nullptr, true,
                                                      surface_area);
  if (!std::isfinite(energy)) return false;

  max_iterations = std::max(1, max_iterations);
  for (int iter = 0; iter < max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> grad;
    energy = symdirichlet::meshEnergyAndGradient(V, F, UV, &grad, true, surface_area);
    if (!std::isfinite(energy)) break;
    for (int vi : fixed) {
      if (vi >= 0 && vi < static_cast<int>(grad.size())) grad[vi].setZero();
    }

    double max_grad = 0.0;
    for (int vi = 0; vi < UV.rows(); ++vi) {
      if (fixed.count(vi) != 0) continue;
      max_grad = std::max(max_grad, grad[vi].norm());
    }
    if (max_grad < 1.0e-10) return true;

    const Eigen::MatrixXd old_uv = UV;
    double step = 0.25 * chart_scale / max_grad;
    bool accepted = false;
    for (int ls = 0; ls < 16; ++ls) {
      UV = old_uv;
      for (int vi = 0; vi < UV.rows(); ++vi) {
        if (fixed.count(vi) != 0) continue;
        UV.row(vi) -= step * grad[vi].transpose();
      }
      const double trial = symdirichlet::meshEnergyAndGradient(V, F, UV, nullptr, true,
                                                               surface_area);
      if (std::isfinite(trial) && trial < energy) {
        energy = trial;
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) break;
  }
  return std::isfinite(energy);
}

}  // namespace meshparam

#endif  // _MESHPARAM_HXX
