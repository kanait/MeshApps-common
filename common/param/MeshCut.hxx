////////////////////////////////////////////////////////////////////
//
// Open closed triangle meshes with a farthest-point seam cut
// (optcuts-style initial cut for parameterization).
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _MESHCUT_HXX
#define _MESHCUT_HXX 1

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "myEigen.hxx"

namespace meshcut {

inline std::pair<int, int> edgeKey(int a, int b) {
  return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

inline bool meshHasBoundary(const Eigen::MatrixXi& F) {
  std::map<std::pair<int, int>, int> count;
  for (int fi = 0; fi < F.rows(); ++fi) {
    for (int c = 0; c < 3; ++c) {
      const int a = F(fi, c);
      const int b = F(fi, (c + 1) % 3);
      ++count[edgeKey(a, b)];
    }
  }
  for (const auto& kv : count) {
    if (kv.second == 1) return true;
  }
  return false;
}

inline std::vector<std::vector<std::pair<int, double>>> buildVertexAdjacency(
    int n, const Eigen::MatrixXi& F) {
  std::vector<std::vector<std::pair<int, double>>> adj(static_cast<size_t>(n));
  std::set<std::pair<int, int>> seen;
  for (int fi = 0; fi < F.rows(); ++fi) {
    for (int c = 0; c < 3; ++c) {
      const int a = F(fi, c);
      const int b = F(fi, (c + 1) % 3);
      if (a < 0 || b < 0 || a >= n || b >= n || a == b) continue;
      const auto key = edgeKey(a, b);
      if (!seen.insert(key).second) continue;
      const double len = 1.0;  // combinatorial geodesic
      adj[static_cast<size_t>(a)].emplace_back(b, len);
      adj[static_cast<size_t>(b)].emplace_back(a, len);
    }
  }
  return adj;
}

inline std::vector<double> dijkstra(
    const std::vector<std::vector<std::pair<int, double>>>& adj, int start,
    std::vector<int>* out_prev = nullptr) {
  const int n = static_cast<int>(adj.size());
  std::vector<double> dist(static_cast<size_t>(n),
                           std::numeric_limits<double>::infinity());
  std::vector<int> prev(static_cast<size_t>(n), -1);
  using Node = std::pair<double, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  dist[static_cast<size_t>(start)] = 0.0;
  pq.push({0.0, start});
  while (!pq.empty()) {
    const auto [d, v] = pq.top();
    pq.pop();
    if (d != dist[static_cast<size_t>(v)]) continue;
    for (const auto& nb : adj[static_cast<size_t>(v)]) {
      const double nd = d + nb.second;
      if (nd < dist[static_cast<size_t>(nb.first)]) {
        dist[static_cast<size_t>(nb.first)] = nd;
        prev[static_cast<size_t>(nb.first)] = v;
        pq.push({nd, nb.first});
      }
    }
  }
  if (out_prev) *out_prev = prev;
  return dist;
}

inline int farthestFiniteVertex(const std::vector<double>& dist) {
  int best = -1;
  double best_d = -1.0;
  for (int i = 0; i < static_cast<int>(dist.size()); ++i) {
    if (!std::isfinite(dist[static_cast<size_t>(i)])) continue;
    if (dist[static_cast<size_t>(i)] > best_d) {
      best_d = dist[static_cast<size_t>(i)];
      best = i;
    }
  }
  return best;
}

inline std::vector<int> reconstructPath(int start, int goal,
                                        const std::vector<int>& prev) {
  std::vector<int> path;
  if (start < 0 || goal < 0 || start >= static_cast<int>(prev.size()) ||
      goal >= static_cast<int>(prev.size())) {
    return path;
  }
  for (int v = goal; v >= 0; v = prev[static_cast<size_t>(v)]) {
    path.push_back(v);
    if (v == start) break;
  }
  if (path.empty() || path.back() != start) return {};
  std::reverse(path.begin(), path.end());
  return path;
}

// Duplicate vertices along a geodesic path so the mesh has a boundary.
// vtx_original maps each new vertex index to its source index in the input V.
inline bool meshFarthestPointCut(Eigen::MatrixXd& V, Eigen::MatrixXi& F,
                                 std::vector<int>* vtx_original = nullptr) {
  if (meshHasBoundary(F)) {
    if (vtx_original) {
      vtx_original->resize(V.rows());
      std::iota(vtx_original->begin(), vtx_original->end(), 0);
    }
    return true;
  }

  const int n = static_cast<int>(V.rows());
  if (n < 4 || F.rows() < 2) return false;

  const auto adj = buildVertexAdjacency(n, F);
  std::vector<int> prev;
  std::vector<double> dist = dijkstra(adj, 0, &prev);
  const int a = farthestFiniteVertex(dist);
  if (a < 0) return false;

  dist = dijkstra(adj, a, &prev);
  const int b = farthestFiniteVertex(dist);
  if (b < 0 || a == b) return false;

  const std::vector<int> path = reconstructPath(a, b, prev);
  if (path.size() < 2) return false;

  std::set<int> path_vertices(path.begin(), path.end());
  std::set<std::pair<int, int>> seam_edges;
  for (size_t i = 1; i < path.size(); ++i) {
    seam_edges.insert(edgeKey(path[i - 1], path[i]));
  }

  std::map<std::pair<int, int>, std::vector<int>> edge_faces;
  for (int fi = 0; fi < F.rows(); ++fi) {
    for (int c = 0; c < 3; ++c) {
      const int e0 = F(fi, c);
      const int e1 = F(fi, (c + 1) % 3);
      edge_faces[edgeKey(e0, e1)].push_back(fi);
    }
  }

  std::vector<std::vector<std::pair<int, std::pair<int, int>>>> face_adj(
      static_cast<size_t>(F.rows()));
  for (const auto& item : edge_faces) {
    const auto& faces_on_edge = item.second;
    for (size_t i = 0; i < faces_on_edge.size(); ++i) {
      for (size_t j = i + 1; j < faces_on_edge.size(); ++j) {
        face_adj[static_cast<size_t>(faces_on_edge[i])].emplace_back(
            faces_on_edge[j], item.first);
        face_adj[static_cast<size_t>(faces_on_edge[j])].emplace_back(
            faces_on_edge[i], item.first);
      }
    }
  }

  // Sheet assignment: flip across seam edges (branch cut), keep across interior edges.
  std::vector<int> sheet(static_cast<size_t>(F.rows()), -1);
  std::queue<std::pair<int, int>> q;
  sheet[0] = 0;
  q.push({0, 0});
  while (!q.empty()) {
    const auto [f, s] = q.front();
    q.pop();
    for (const auto& nb : face_adj[static_cast<size_t>(f)]) {
      const int nf = nb.first;
      const int ns =
          (seam_edges.count(nb.second) != 0) ? (1 - s) : s;
      if (sheet[static_cast<size_t>(nf)] < 0) {
        sheet[static_cast<size_t>(nf)] = ns;
        q.push({nf, ns});
      }
    }
  }
  for (int fi = 0; fi < F.rows(); ++fi) {
    if (sheet[static_cast<size_t>(fi)] < 0) sheet[static_cast<size_t>(fi)] = 0;
  }

  std::map<int, int> dup_index;
  Eigen::MatrixXd Vnew = V;
  for (const int vid : path) {
    const int ni = Vnew.rows();
    Vnew.conservativeResize(ni + 1, 3);
    Vnew.row(ni) = V.row(vid);
    dup_index[vid] = ni;
  }

  Eigen::MatrixXi Fnew = F;
  for (int fi = 0; fi < F.rows(); ++fi) {
    if (sheet[static_cast<size_t>(fi)] != 1) continue;
    for (int c = 0; c < 3; ++c) {
      const int vid = F(fi, c);
      if (path_vertices.count(vid) != 0) {
        Fnew(fi, c) = dup_index.at(vid);
      }
    }
  }

  if (!meshHasBoundary(Fnew)) return false;

  V = Vnew;
  F = Fnew;
  if (vtx_original) {
    vtx_original->resize(V.rows());
    for (int i = 0; i < n; ++i) (*vtx_original)[i] = i;
    for (const auto& item : dup_index) {
      (*vtx_original)[item.second] = item.first;
    }
  }
  return true;
}

inline void collapseUvAlongCut(const Eigen::MatrixXd& UV_cut,
                             const std::vector<int>& vtx_original, int orig_n,
                             Eigen::MatrixXd& UV) {
  UV.resize(orig_n, 2);
  UV.setZero();
  std::vector<char> seen(static_cast<size_t>(orig_n), 0);
  for (int i = 0; i < UV_cut.rows(); ++i) {
    const int orig = vtx_original[static_cast<size_t>(i)];
    if (orig < 0 || orig >= orig_n) continue;
    if (!seen[static_cast<size_t>(orig)]) {
      UV.row(orig) = UV_cut.row(i);
      seen[static_cast<size_t>(orig)] = 1;
    } else {
      UV.row(orig) = 0.5 * (UV.row(orig) + UV_cut.row(i));
    }
  }
}

}  // namespace meshcut

#endif  // _MESHCUT_HXX
