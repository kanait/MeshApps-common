////////////////////////////////////////////////////////////////////
//
// Eigen wrapper for Jonathan Shewchuk's Triangle (2D constrained meshing)
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _TRIANGLETRIANGULATE_HXX
#define _TRIANGLETRIANGULATE_HXX

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <Eigen/Core>

#include "TriangleFpGuard.hxx"
#include "triangle.h"

namespace triangle_mesh {

inline void triangulate(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E,
                        const Eigen::MatrixXd& H, const std::string& flags,
                        Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out) {
  V_out.resize(0, 2);
  F_out.resize(0, 3);
  if (V.cols() != 2 || V.rows() < 3) return;
  if (E.size() > 0 && E.cols() != 2) return;
  if (H.size() > 0 && H.cols() != 2) return;

  // Exact predicates need default IEEE rounding / denormals (see TriangleFpGuard).
  TriangleFpGuard fp_guard;

  const std::string full_flags = flags + "pzQ";

  triangulateio in{};
  std::memset(&in, 0, sizeof(in));
  triangulateio out{};
  std::memset(&out, 0, sizeof(out));

  in.numberofpoints = static_cast<int>(V.rows());
  in.pointlist = static_cast<TRI_REAL*>(
      std::malloc(static_cast<size_t>(2 * V.rows()) * sizeof(TRI_REAL)));
  if (!in.pointlist) return;
  for (int i = 0; i < V.rows(); ++i) {
    in.pointlist[2 * i] = static_cast<TRI_REAL>(V(i, 0));
    in.pointlist[2 * i + 1] = static_cast<TRI_REAL>(V(i, 1));
  }

  in.numberofsegments = E.rows() > 0 ? static_cast<int>(E.rows()) : 0;
  if (in.numberofsegments > 0) {
    in.segmentlist = static_cast<int*>(
        std::malloc(static_cast<size_t>(2 * E.rows()) * sizeof(int)));
    if (!in.segmentlist) {
      std::free(in.pointlist);
      return;
    }
    for (int i = 0; i < E.rows(); ++i) {
      in.segmentlist[2 * i] = E(i, 0);
      in.segmentlist[2 * i + 1] = E(i, 1);
    }
  }

  in.numberofholes = H.rows() > 0 ? static_cast<int>(H.rows()) : 0;
  if (in.numberofholes > 0) {
    in.holelist = static_cast<TRI_REAL*>(
        std::malloc(static_cast<size_t>(2 * H.rows()) * sizeof(TRI_REAL)));
    if (!in.holelist) {
      std::free(in.pointlist);
      std::free(in.segmentlist);
      return;
    }
    for (int i = 0; i < H.rows(); ++i) {
      in.holelist[2 * i] = static_cast<TRI_REAL>(H(i, 0));
      in.holelist[2 * i + 1] = static_cast<TRI_REAL>(H(i, 1));
    }
  }

  char* switches = static_cast<char*>(std::malloc(full_flags.size() + 1));
  if (!switches) {
    std::free(in.pointlist);
    std::free(in.segmentlist);
    std::free(in.holelist);
    return;
  }
  std::snprintf(switches, full_flags.size() + 1, "%s", full_flags.c_str());
  ::triangulate(switches, &in, &out, nullptr);
  std::free(switches);

  // Inputs we allocated with malloc.
  std::free(in.pointlist);
  std::free(in.segmentlist);
  std::free(in.holelist);

  if (out.numberofpoints > 0 && out.pointlist && out.numberoftriangles > 0 &&
      out.trianglelist) {
    V_out.resize(out.numberofpoints, 2);
    for (int i = 0; i < out.numberofpoints; ++i) {
      V_out(i, 0) = out.pointlist[2 * i];
      V_out(i, 1) = out.pointlist[2 * i + 1];
    }
    F_out.resize(out.numberoftriangles, 3);
    for (int i = 0; i < out.numberoftriangles; ++i) {
      F_out(i, 0) = out.trianglelist[3 * i];
      F_out(i, 1) = out.trianglelist[3 * i + 1];
      F_out(i, 2) = out.trianglelist[3 * i + 2];
    }
  }

  // Triangle-allocated arrays must be released with trifree.
  if (out.pointlist) trifree(reinterpret_cast<int*>(out.pointlist));
  if (out.trianglelist) trifree(reinterpret_cast<int*>(out.trianglelist));
  if (out.pointmarkerlist) trifree(reinterpret_cast<int*>(out.pointmarkerlist));
  if (out.segmentlist) trifree(reinterpret_cast<int*>(out.segmentlist));
  if (out.segmentmarkerlist) trifree(reinterpret_cast<int*>(out.segmentmarkerlist));
  if (out.neighborlist) trifree(reinterpret_cast<int*>(out.neighborlist));
  if (out.edgelist) trifree(reinterpret_cast<int*>(out.edgelist));
  if (out.edgemarkerlist) trifree(reinterpret_cast<int*>(out.edgemarkerlist));
}

}  // namespace triangle_mesh

#endif  // _TRIANGLETRIANGULATE_HXX
