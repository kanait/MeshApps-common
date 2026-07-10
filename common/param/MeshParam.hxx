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

}  // namespace meshparam

#endif  // _MESHPARAM_HXX
