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

#include <cstdlib>
#include <string>

#include <Eigen/Core>

#include "triangle.h"

namespace triangle_mesh {

inline void triangulate(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E,
                        const Eigen::MatrixXd& H, const std::string& flags,
                        Eigen::MatrixXd& V_out, Eigen::MatrixXi& F_out) {
  if (V.cols() != 2) return;
  if (E.size() > 0 && E.cols() != 2) return;
  if (H.size() > 0 && H.cols() != 2) return;

  const std::string full_flags = flags + "pzQ";

  triangulateio in{};
  in.numberofpoints = static_cast<int>(V.rows());
  in.pointlist =
      static_cast<TRI_REAL*>(std::malloc(static_cast<size_t>(V.size()) *
                                         sizeof(TRI_REAL)));
  for (int i = 0; i < V.rows(); ++i) {
    in.pointlist[2 * i] = V(i, 0);
    in.pointlist[2 * i + 1] = V(i, 1);
  }

  in.numberofpointattributes = 0;
  in.pointmarkerlist = nullptr;

  in.numberofsegments = E.rows() > 0 ? static_cast<int>(E.rows()) : 0;
  if (in.numberofsegments > 0) {
    in.segmentlist = static_cast<int*>(std::malloc(
        static_cast<size_t>(E.size()) * sizeof(int)));
    for (int i = 0; i < E.rows(); ++i) {
      in.segmentlist[2 * i] = E(i, 0);
      in.segmentlist[2 * i + 1] = E(i, 1);
    }
  } else {
    in.segmentlist = nullptr;
  }
  in.segmentmarkerlist = nullptr;

  in.numberofholes = H.rows() > 0 ? static_cast<int>(H.rows()) : 0;
  if (in.numberofholes > 0) {
    in.holelist = static_cast<TRI_REAL*>(std::malloc(
        static_cast<size_t>(H.size()) * sizeof(TRI_REAL)));
    for (int i = 0; i < H.rows(); ++i) {
      in.holelist[2 * i] = H(i, 0);
      in.holelist[2 * i + 1] = H(i, 1);
    }
  } else {
    in.holelist = nullptr;
  }
  in.numberofregions = 0;

  in.trianglelist = nullptr;
  in.numberoftriangles = 0;
  in.numberofcorners = 0;
  in.numberoftriangleattributes = 0;
  in.triangleattributelist = nullptr;

  triangulateio out{};
  out.pointlist = nullptr;
  out.trianglelist = nullptr;

  char* switches =
      static_cast<char*>(std::malloc(full_flags.size() + 1));
  std::snprintf(switches, full_flags.size() + 1, "%s", full_flags.c_str());
  ::triangulate(switches, &in, &out, nullptr);
  std::free(switches);

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

  std::free(in.pointlist);
  std::free(in.segmentlist);
  std::free(in.holelist);
  std::free(out.pointlist);
  std::free(out.trianglelist);
}

}  // namespace triangle_mesh

#endif  // _TRIANGLETRIANGULATE_HXX
