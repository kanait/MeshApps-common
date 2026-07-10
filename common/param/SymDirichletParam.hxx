////////////////////////////////////////////////////////////////////
//
// Symmetric Dirichlet UV parameterization for MeshL.
//
// Tutte (ARAP) initialization + gradient descent on symmetric Dirichlet
// energy (optcuts-style chart relaxation, without scaffold).
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _SYMDIRICHLET_PARAM_HXX
#define _SYMDIRICHLET_PARAM_HXX 1

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "ArapParam.hxx"
#include "LscmParam.hxx"
#include "MeshL.hxx"
#include "MeshUtiL.hxx"
#include "SymDirichletEnergy.hxx"

class SymDirichletParam {
 public:
  SymDirichletParam() : mesh_(nullptr) {}

  explicit SymDirichletParam(std::shared_ptr<MeshL> mesh) : mesh_(mesh) {}

  void setMesh(std::shared_ptr<MeshL> mesh) { mesh_ = mesh; }
  void setMaxIterations(int n) { max_iterations_ = std::max(1, n); }
  void setVerbose(bool verbose) { verbose_ = verbose; }
  void setQuiet(bool quiet) { quiet_ = quiet; }
  // OptCuts-style: when true, only one vertex is pinned during relaxation.
  void setFreeBoundary(bool free_boundary) { free_boundary_ = free_boundary; }
  // Legacy alias: pin_boundary=false means free_boundary=true.
  void setPinBoundary(bool pin_boundary) { free_boundary_ = !pin_boundary; }

  bool apply() {
    if (!mesh_) {
      if (!quiet_) std::cerr << "SymDirichletParam: mesh is null." << std::endl;
      return false;
    }

    if (!initializeUv()) {
      if (!quiet_) {
        std::cerr << "SymDirichletParam: initialization failed." << std::endl;
      }
      return false;
    }

    mesh_->createConnectivity(true);
    indexVertices();
    if (!buildEigenMesh()) return false;
    if (!importUvFromMesh()) return false;
    fixOrientationAndNormalize();

    std::set<int> fixed;
    collectFixedPins(fixed);

    const Eigen::MatrixXd init_uv = UV_;
    const double init_energy = symdirichlet::meshEnergyAndGradient(
        V_, F_, init_uv, nullptr, true, surface_area_);
    const bool relaxed = relaxSymDirichlet(fixed);
    const double final_energy = symdirichlet::meshEnergyAndGradient(
        V_, F_, UV_, nullptr, true, surface_area_);
    if (!relaxed || !std::isfinite(final_energy) ||
        (std::isfinite(init_energy) && final_energy > init_energy)) {
      UV_ = init_uv;
      if (!quiet_ && relaxed && std::isfinite(init_energy) &&
          std::isfinite(final_energy) && final_energy > init_energy) {
        std::cerr << "SymDirichletParam: reverted to initialization UV."
                  << std::endl;
      }
    }

    assignTexcoordsFromEigen();
    if (verbose_) {
      std::cout << "symdirichlet: done. v " << mesh_->vertices_size() << " t "
                << mesh_->texcoords().size() << std::endl;
    }
    return true;
  }

 private:
  std::shared_ptr<MeshL> mesh_;
  int max_iterations_ = 80;
  bool verbose_ = false;
  bool quiet_ = true;
  bool free_boundary_ = true;

  std::map<std::shared_ptr<VertexL>, int> vtx_index_;
  Eigen::MatrixXd V_;
  Eigen::MatrixXi F_;
  Eigen::MatrixXd UV_;
  double surface_area_ = 1.0;
  double avg_edge_length_ = 1.0;

  bool initializeUv() {
    ArapParam arap(mesh_);
    arap.setInitMode(ArapParam::InitMode::Tutte);
    arap.setVerbose(false);
    arap.setQuiet(true);
    if (arap.apply()) return true;

    LscmParam lscm(mesh_);
    lscm.setVerbose(false);
    lscm.setQuiet(true);
    return lscm.apply();
  }

  void indexVertices() {
    vtx_index_.clear();
    int i = 0;
    for (const auto& vt : mesh_->vertices()) vtx_index_[vt] = i++;
  }

  bool buildEigenMesh() {
    const int n = static_cast<int>(mesh_->vertices_size());
    V_.resize(n, 3);
    int i = 0;
    for (const auto& vt : mesh_->vertices()) {
      V_.row(i++) = vt->point().transpose();
    }

    F_.resize(static_cast<int>(mesh_->faces_size()), 3);
    int fi = 0;
    for (const auto& fc : mesh_->faces()) {
      std::vector<std::shared_ptr<HalfedgeL>> he(fc->halfedges().begin(),
                                                fc->halfedges().end());
      if (he.size() != 3) return false;
      for (int c = 0; c < 3; ++c) {
        auto it = vtx_index_.find(he[c]->vertex());
        if (it == vtx_index_.end()) return false;
        F_(fi, c) = it->second;
      }
      ++fi;
    }

    surface_area_ = symdirichlet::meshSurfaceArea(V_, F_);
    double edge_sum = 0.0;
    int edge_count = 0;
    for (int f = 0; f < F_.rows(); ++f) {
      for (int e = 0; e < 3; ++e) {
        const int a = F_(f, e);
        const int b = F_(f, (e + 1) % 3);
        edge_sum += (V_.row(a) - V_.row(b)).norm();
        ++edge_count;
      }
    }
    avg_edge_length_ = edge_count > 0 ? edge_sum / edge_count : 1.0;
    return true;
  }

  bool importUvFromMesh() {
    UV_.resize(V_.rows(), 2);
    UV_.setZero();
    for (const auto& fc : mesh_->faces()) {
      for (const auto& he : fc->halfedges()) {
        const auto tc = he->texcoord();
        if (!tc) continue;
        auto it = vtx_index_.find(he->vertex());
        if (it == vtx_index_.end()) continue;
        UV_.row(it->second) = tc->point().head<2>().transpose();
      }
    }
    return true;
  }

  void fixOrientationAndNormalize() {
    double signed_area = 0.0;
    for (int fi = 0; fi < F_.rows(); ++fi) {
      const int i0 = F_(fi, 0);
      const int i1 = F_(fi, 1);
      const int i2 = F_(fi, 2);
      signed_area += symdirichlet::cross2(UV_.row(i1) - UV_.row(i0),
                                          UV_.row(i2) - UV_.row(i0));
    }
    if (signed_area < 0.0) UV_.col(1) *= -1.0;

    const double umin = UV_.col(0).minCoeff();
    const double umax = UV_.col(0).maxCoeff();
    const double vmin = UV_.col(1).minCoeff();
    const double vmax = UV_.col(1).maxCoeff();
    const double span = std::max(umax - umin, vmax - vmin);
    if (span < 1.0e-12) return;
    for (int i = 0; i < UV_.rows(); ++i) {
      UV_(i, 0) = (UV_(i, 0) - umin) / span;
      UV_(i, 1) = (UV_(i, 1) - vmin) / span;
    }
  }

  void collectFixedPins(std::set<int>& fixed) const {
    fixed.clear();
    if (free_boundary_) {
      if (UV_.rows() > 0) fixed.insert(0);
      return;
    }
    std::vector<std::shared_ptr<VertexL>> boundary;
    mesh_->createConnectivity(true);
    if (collectLongestBoundaryLoop(mesh_->halfedges(), boundary)) {
      for (const auto& vt : boundary) {
        auto it = vtx_index_.find(vt);
        if (it != vtx_index_.end()) fixed.insert(it->second);
      }
    }
    if (fixed.empty() && UV_.rows() > 0) fixed.insert(0);
  }

  bool relaxSymDirichlet(const std::set<int>& fixed) {
    double energy = symdirichlet::meshEnergyAndGradient(
        V_, F_, UV_, nullptr, true, surface_area_);
    if (!std::isfinite(energy)) return false;

    const double chart_scale =
        std::max(std::sqrt(std::max(surface_area_, symdirichlet::kEps)), avg_edge_length_);

    for (int iter = 0; iter < max_iterations_; ++iter) {
      std::vector<Eigen::Vector2d> grad;
      energy = symdirichlet::meshEnergyAndGradient(V_, F_, UV_, &grad, true,
                                                   surface_area_);
      if (!std::isfinite(energy)) break;
      for (int vi : fixed) {
        if (vi >= 0 && vi < static_cast<int>(grad.size())) grad[vi].setZero();
      }

      double max_grad = 0.0;
      for (int vi = 0; vi < UV_.rows(); ++vi) {
        if (fixed.count(vi) != 0) continue;
        max_grad = std::max(max_grad, grad[vi].norm());
      }
      if (max_grad < 1.0e-10) return true;

      Eigen::MatrixXd old_uv = UV_;
      double step = 0.25 * chart_scale / max_grad;
      bool accepted = false;
      for (int ls = 0; ls < 16; ++ls) {
        UV_ = old_uv;
        for (int vi = 0; vi < UV_.rows(); ++vi) {
          if (fixed.count(vi) != 0) continue;
          UV_.row(vi) -= step * grad[vi].transpose();
        }
        const double trial = symdirichlet::meshEnergyAndGradient(
            V_, F_, UV_, nullptr, true, surface_area_);
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

  void assignTexcoordsFromEigen() {
    mesh_->deleteAllTexcoords();
    mesh_->createConnectivity(true);
    for (const auto& fc : mesh_->faces()) {
      std::vector<std::shared_ptr<HalfedgeL>> hes(fc->halfedges().begin(),
                                                  fc->halfedges().end());
      if (hes.size() != 3) continue;
      for (auto& he : hes) {
        auto it = vtx_index_.find(he->vertex());
        if (it == vtx_index_.end()) continue;
        const Eigen::Vector2d uv = UV_.row(it->second);
        Eigen::Vector3d p(uv.x(), uv.y(), 0.0);
        he->setTexcoord(mesh_->addTexcoord(p));
      }
    }
  }
};

#endif  // _SYMDIRICHLET_PARAM_HXX
