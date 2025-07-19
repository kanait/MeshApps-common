////////////////////////////////////////////////////////////////////
//
// $Id: EulerOperations.hxx 2025/07/05 18:39:35 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _EULEROPERATIONS_HXX
#define _EULEROPERATIONS_HXX 1

#include "envDep.h"
#include "mydef.h"
#include "myEigen.hxx"
#include "MeshL.hxx"
#include <map>
#include <set>

class EulerOperations {
 public:
  // Constructor
  EulerOperations(std::shared_ptr<MeshL> mesh) : mesh_(mesh) {}

  // ============================================================================
  // Rigorous Half-edge Connectivity Management
  // ============================================================================

  // Helper function to properly set mate relationships
  void setMate(std::shared_ptr<HalfedgeL> he1, std::shared_ptr<HalfedgeL> he2) {
    if (he1 && he2) {
      he1->setMate(he2);
      he2->setMate(he1);
    }
  }

  // Helper function to update vertex's halfedge pointer
  void updateVertexHalfedge(std::shared_ptr<VertexL> vertex,
                            std::shared_ptr<HalfedgeL> he) {
    if (vertex && he) {
      vertex->setHalfedge(he);
    }
  }

  // Helper function to ensure all vertices have halfedge pointers
  void ensureVertexHalfedges(std::shared_ptr<MeshL> mesh) {
    if (!mesh) return;

    for (auto vertex : mesh->vertices()) {
      if (!vertex->halfedge()) {
        // Find a halfedge that points to this vertex
        bool found = false;
        for (auto face : mesh->faces()) {
          for (auto he : face->halfedges()) {
            if (he->vertex() == vertex) {
              vertex->setHalfedge(he);
              found = true;
              break;
            }
          }
          if (found) break;
        }
      }
    }
  }

  // Update all mate relationships (preserving existing mates)
  void updateAllMates() {
    if (!mesh_) return;
    
    // First, preserve existing mate relationships that are still valid
    std::map<int, int> preserved_mates;
    for (auto face : mesh_->faces()) {
      for (auto he : face->halfedges()) {
        if (he->mate()) {
          preserved_mates[he->id()] = he->mate()->id();
        }
      }
    }
    
    // Clear all existing mate relationships
    for (auto face : mesh_->faces()) {
      for (auto he : face->halfedges()) {
        he->setMate(nullptr);
      }
    }
    
    // Restore preserved mate relationships
    for (auto face : mesh_->faces()) {
      for (auto he : face->halfedges()) {
        auto it = preserved_mates.find(he->id());
        if (it != preserved_mates.end()) {
          // Find the mate halfedge
          for (auto face2 : mesh_->faces()) {
            for (auto he2 : face2->halfedges()) {
              if (he2->id() == it->second) {
                he->setMate(he2);
                he2->setMate(he);
                break;
              }
            }
          }
        }
      }
    }
    
    // Find and set new mate relationships for unmatched halfedges
    for (auto face1 : mesh_->faces()) {
      for (auto he1 : face1->halfedges()) {
        if (he1->mate()) continue; // Already has mate
        auto v1 = he1->vertex();
        auto v2 = he1->next()->vertex();
        
        // Look for mate in other faces
        for (auto face2 : mesh_->faces()) {
          if (face1 == face2) continue;
          for (auto he2 : face2->halfedges()) {
            if (he2->mate()) continue;
            auto v2_he2 = he2->vertex();
            auto v1_he2 = he2->next()->vertex();
            if ((v1 == v2_he2 && v2 == v1_he2) || (v1 == v1_he2 && v2 == v2_he2)) {
              he1->setMate(he2);
              he2->setMate(he1);
              break;
            }
          }
          if (he1->mate()) break;
        }
      }
    }
  }

  // Helper function to rebuild face's halfedge list in correct order (strict)
  void rebuildFaceHalfedges(
      std::shared_ptr<FaceL> face,
      const std::vector<std::shared_ptr<HalfedgeL>>& halfedges) {
    if (!face) {
      return;
    }
    
    // Safety check: ensure all halfedges are valid
    for (auto& he : halfedges) {
      if (!he) {
        return;
      }
      if (!he->vertex()) {
        return;
      }
    }
    
    face->deleteHalfedges();
    std::set<int> seen;
    for (auto& he : halfedges) {
      if (he && seen.insert(he->id()).second) {
        face->addHalfedge(he);
      }
    }
    
    // f_iter, f_hlistをセット
    auto& hlist = face->halfedges();
    for (auto iter = hlist.begin(); iter != hlist.end(); ++iter) {
      if (*iter) {
        (*iter)->setFIter(iter);
        (*iter)->setFHalfedges(hlist);
      }
    }
    
    // 循環性チェック
    if (!hlist.empty()) {
      auto start = *hlist.begin();
      if (!start) {
        return;
      }
      auto he = start;
      int count = 0;
      do {
        he = he->next();
        count++;
        if (!he || count > (int)hlist.size()) {
          break;
        }
      } while (he != start);
    }
  }

  // ============================================================================
  // Basic Euler Operations with Rigorous Connectivity
  // ============================================================================

  // OpenMesh-style MEV: Split an internal edge and create a new vertex
  static std::shared_ptr<VertexL> makeEdgeVertexOpenMesh(
      std::shared_ptr<MeshL> mesh,
      std::shared_ptr<VertexL> v1,
      std::shared_ptr<VertexL> v2,
      const Eigen::Vector3d& newPos) {
    
    if (!mesh || !v1 || !v2) {
      return nullptr;
    }

    // Find the internal edge between v1 and v2
    std::shared_ptr<HalfedgeL> he1 = nullptr;
    std::shared_ptr<HalfedgeL> he2 = nullptr;
    
    for (auto face : mesh->faces()) {
      for (auto he : face->halfedges()) {
        if (he->vertex() == v1 && he->mate() && he->mate()->vertex() == v2) {
          he1 = he;
          he2 = he->mate();
          break;
        }
      }
      if (he1 && he2) break;
    }
    
    if (!he1 || !he2) {
      return nullptr;
    }

    // Check for self-loop edge (same face on both sides)
    if (he1->face() == he2->face()) {
      return nullptr;
    }

    // Create new vertex
    Eigen::Vector3d pos_copy = newPos;  // Create non-const copy
    auto newVertex = mesh->addVertex(pos_copy);
    newVertex->setID(mesh->vertices().size() - 1);

    // Create new halfedges for the split internal edge
    auto he1_new = mesh->addHalfedge(he1->face(), v1);
    auto he2_new = mesh->addHalfedge(he2->face(), v2);
    auto he_new_1 = mesh->addHalfedge(he1->face(), newVertex);
    auto he_new_2 = mesh->addHalfedge(he2->face(), newVertex);

    // Set mate relationships for the new internal edge (maintaining internal edge)
    he1_new->setMate(he2_new);
    he2_new->setMate(he1_new);
    
    // face参照を確実に設定
    he1_new->setFace(he1->face());
    he2_new->setFace(he2->face());
    he_new_1->setFace(he1->face());
    he_new_2->setFace(he2->face());
    
    // vertex参照を確実に設定
    he1_new->setVertex(v1);
    he2_new->setVertex(v2);
    he_new_1->setVertex(newVertex);
    he_new_2->setVertex(newVertex);
    

    // Clear mate relationships of original edge (to be removed)
    he1->setMate(nullptr);
    he2->setMate(nullptr);

    // Rebuild face halfedge lists
    auto face1 = he1->face();
    auto face2 = he2->face();
    
    std::vector<std::shared_ptr<HalfedgeL>> face1_halfedges;
    std::vector<std::shared_ptr<HalfedgeL>> face2_halfedges;
    
    // Collect halfedges for face1 (excluding the original edge)
    for (auto he : face1->halfedges()) {
      if (he != he1 && he != he2) {
        face1_halfedges.push_back(he);
      }
    }
    face1_halfedges.push_back(he1_new);
    face1_halfedges.push_back(he_new_1);
    
    // Collect halfedges for face2 (excluding the original edge)
    for (auto he : face2->halfedges()) {
      if (he != he1 && he != he2) {
        face2_halfedges.push_back(he);
      }
    }
    face2_halfedges.push_back(he2_new);
    face2_halfedges.push_back(he_new_2);

    // Remove original halfedges from mesh
    mesh->deleteHalfedge(he1);
    mesh->deleteHalfedge(he2);

    // Create temporary EulerOperations instance for rebuildFaceHalfedges
    EulerOperations temp_euler(mesh);
    // --- face halfedgeリストをクリアする ---
    face1->deleteHalfedges();
    face2->deleteHalfedges();
    // --- 新halfedgeリストのみをセット ---
    temp_euler.rebuildFaceHalfedges(face1, face1_halfedges);
    temp_euler.rebuildFaceHalfedges(face2, face2_halfedges);

    // Update vertex halfedge pointers
    temp_euler.ensureVertexHalfedges(mesh);
    EulerOperations temp_euler2(mesh);
    temp_euler2.ensureVertexHalfedges(mesh);

    
    return newVertex;
  }

  // OpenMesh-style MEF: Split a face by creating a new edge between two vertices
  static std::shared_ptr<FaceL> makeEdgeFaceOpenMesh(
      std::shared_ptr<MeshL> mesh,
      std::shared_ptr<VertexL> v1,
      std::shared_ptr<VertexL> v2) {
    
    if (!mesh || !v1 || !v2) {
      return nullptr;
    }

    // Find a face that contains both vertices
    std::shared_ptr<FaceL> targetFace = nullptr;
    std::shared_ptr<HalfedgeL> he_v1 = nullptr;
    std::shared_ptr<HalfedgeL> he_v2 = nullptr;
    
    for (auto face : mesh->faces()) {
      bool found_v1 = false, found_v2 = false;
      for (auto he : face->halfedges()) {
        if (he->vertex() == v1) {
          he_v1 = he;
          found_v1 = true;
        }
        if (he->vertex() == v2) {
          he_v2 = he;
          found_v2 = true;
        }
      }
      if (found_v1 && found_v2) {
        targetFace = face;
        break;
      }
    }
    
    if (!targetFace) {
      return nullptr;
    }

    // Find indices of he_v1 and he_v2 in the face
    int idx_v1 = -1, idx_v2 = -1;
    int idx = 0;
    for (auto he : targetFace->halfedges()) {
      if (he == he_v1) idx_v1 = idx;
      if (he == he_v2) idx_v2 = idx;
      idx++;
    }
    

    // Create new face
    auto newFace = mesh->addFace();
    newFace->setID(mesh->faces().size() - 1);

    // Create new halfedges for the edge
    auto he1 = mesh->addHalfedge(targetFace, v1);
    auto he2 = mesh->addHalfedge(newFace, v2);

    // Set mate relationship
    he1->setMate(he2);
    he2->setMate(he1);

    // Split the face halfedges into two paths
    std::vector<std::shared_ptr<HalfedgeL>> path1, path2;
    std::vector<std::shared_ptr<HalfedgeL>> face_halfedges;
    
    for (auto he : targetFace->halfedges()) {
      face_halfedges.push_back(he);
    }

    // Find the two paths between v1 and v2
    int start_idx = std::min(idx_v1, idx_v2);
    int end_idx = std::max(idx_v1, idx_v2);
    
    // Path 1: from start to end
    for (int i = start_idx; i <= end_idx; i++) {
      path1.push_back(face_halfedges[i]);
    }
    
    // Path 2: from end to start (wrapping around)
    for (int i = end_idx; i < face_halfedges.size(); i++) {
      path2.push_back(face_halfedges[i]);
    }
    for (int i = 0; i <= start_idx; i++) {
      path2.push_back(face_halfedges[i]);
    }
    

    // Add new halfedges to paths
    path1.push_back(he1);
    path2.push_back(he2);

    // Rebuild faces
    EulerOperations temp_euler(mesh);
    // --- face halfedgeリストをクリアする ---
    targetFace->deleteHalfedges();
    newFace->deleteHalfedges();
    // --- 新halfedgeリストのみをセット ---
    temp_euler.rebuildFaceHalfedges(targetFace, path1);
    temp_euler.rebuildFaceHalfedges(newFace, path2);
    
    // --- ここから修正: 新規halfedgeのface参照を正しく設定 ---
    // he1はtargetFaceに属する
    he1->setFace(targetFace);
    // he2はnewFaceに属する
    he2->setFace(newFace);
    // --- ここまで修正 ---

    // Update vertex halfedge pointers
    temp_euler.ensureVertexHalfedges(mesh);
    EulerOperations temp_euler2(mesh);
    temp_euler2.ensureVertexHalfedges(mesh);

    
    return newFace;
  }

  // Boundary Edge MEV - Add a new vertex on a boundary edge
  std::shared_ptr<VertexL> makeEdgeVertex(std::shared_ptr<HalfedgeL> edge_halfedge,
                                          const Eigen::Vector3d& new_pos) {
    if (!mesh_ || !edge_halfedge) return nullptr;

    auto v1 = edge_halfedge->vertex();
    auto v2 = edge_halfedge->next()->vertex();
    if (!v1 || !v2) return nullptr;

    auto face = edge_halfedge->face();
    if (!face) return nullptr;


    // Create new vertex
    Eigen::Vector3d pos_copy = new_pos;  // Create non-const copy
    auto newVertex = mesh_->addVertex(pos_copy);
    newVertex->setID(mesh_->vertices().size() - 1);

    // Create new halfedges for the split edge
    auto he1 = mesh_->addHalfedge(face, v1);
    auto he2 = mesh_->addHalfedge(face, newVertex);

    // Rebuild face halfedge list
    std::vector<std::shared_ptr<HalfedgeL>> new_halfedges;
    for (auto he : face->halfedges()) {
      if (he == edge_halfedge) {
        // Replace the original edge with two new edges
        new_halfedges.push_back(he1);
        new_halfedges.push_back(he2);
      } else {
        new_halfedges.push_back(he);
      }
    }

    rebuildFaceHalfedges(face, new_halfedges);

    // Update vertex halfedge pointers
    updateVertexHalfedge(newVertex, he2);
    ensureVertexHalfedges(mesh_);

    return newVertex;
  }

  // Edge Split MEV - Split an existing edge by inserting a new vertex
  std::shared_ptr<VertexL> splitEdgeMakeVertex(std::shared_ptr<HalfedgeL> edge_halfedge,
                                               const Eigen::Vector3d& new_pos) {
    if (!mesh_ || !edge_halfedge) return nullptr;

    auto v1 = edge_halfedge->vertex();
    auto v2 = edge_halfedge->next()->vertex();
    if (!v1 || !v2) return nullptr;

    auto mate_halfedge = edge_halfedge->mate();
    auto face1 = edge_halfedge->face();
    auto face2 = mate_halfedge ? mate_halfedge->face() : nullptr;

    // Step 1: Adding new vertex with MEV...
    auto newVertex = EulerOperations::makeEdgeVertexOpenMesh(mesh_, v1, v2, new_pos);
    if (!newVertex) {
      return nullptr;
    }

    // face1側のhalfedgeリストを分割
    std::vector<std::shared_ptr<HalfedgeL>> new_halfedges1;
    for (auto he : face1->halfedges()) {
      if (he == edge_halfedge) {
        // v1→new_vertex, new_vertex→v2
        auto he1 = mesh_->addHalfedge(face1, v1);
        auto he2 = mesh_->addHalfedge(face1, newVertex);
        new_halfedges1.push_back(he1);
        new_halfedges1.push_back(he2);
      } else {
        new_halfedges1.push_back(he);
      }
    }
    rebuildFaceHalfedges(face1, new_halfedges1);

    // face2側も同様
    if (face2 && mate_halfedge) {
      std::vector<std::shared_ptr<HalfedgeL>> new_halfedges2;
      for (auto he : face2->halfedges()) {
        if (he == mate_halfedge) {
          auto he3 = mesh_->addHalfedge(face2, v2);
          auto he4 = mesh_->addHalfedge(face2, newVertex);
          new_halfedges2.push_back(he4);
          new_halfedges2.push_back(he3);
        } else {
          new_halfedges2.push_back(he);
        }
      }
      rebuildFaceHalfedges(face2, new_halfedges2);
    }

    // mate関係を厳密に張る
    updateAllMates();

    // 頂点のhalfedgeをセット
    updateVertexHalfedge(newVertex, nullptr);
    ensureVertexHalfedges(mesh_);

    return newVertex;
  }

  // OpenMesh-style MEF: Split a face by inserting an edge between two halfedges
  std::shared_ptr<FaceL> makeEdgeFace(std::shared_ptr<HalfedgeL> he_v1, std::shared_ptr<HalfedgeL> he_v2) {
    if (!mesh_ || !he_v1 || !he_v2) return nullptr;
    auto f = he_v1->face();
    if (!f || he_v1->vertex() == he_v2->vertex()) return nullptr;

    // --- ここから修正: self-loop判定を強化 ---
    // 同じface内で同じvertexを指すhalfedgeがないかチェック
    if (he_v1->vertex() == he_v2->vertex()) {
      return nullptr;
    }
    // --- ここまで修正 ---


    // Get current halfedges in the face
    auto current_halfedges = f->halfedges();
    std::vector<std::shared_ptr<HalfedgeL>> halfedges_list(current_halfedges.begin(), current_halfedges.end());
    
    // Find indices of he_v1 and he_v2 in the list
    int idx_v1 = -1, idx_v2 = -1;
    for (int i = 0; i < halfedges_list.size(); ++i) {
      if (halfedges_list[i] == he_v1) idx_v1 = i;
      if (halfedges_list[i] == he_v2) idx_v2 = i;
    }
    
    if (idx_v1 == -1 || idx_v2 == -1) {
      return nullptr;
    }
    

    // path1: he_v1->next() から he_v2 まで（he_v1, he_v2は含めない）
    std::vector<std::shared_ptr<HalfedgeL>> path1;
    int current_idx = (idx_v1 + 1) % halfedges_list.size();
    while (current_idx != idx_v2) {
      path1.push_back(halfedges_list[current_idx]);
      current_idx = (current_idx + 1) % halfedges_list.size();
      if (path1.size() > halfedges_list.size()) {
        return nullptr;
      }
    }

    // path2: he_v2->next() から he_v1 まで（he_v1, he_v2は含めない）
    std::vector<std::shared_ptr<HalfedgeL>> path2;
    current_idx = (idx_v2 + 1) % halfedges_list.size();
    while (current_idx != idx_v1) {
      path2.push_back(halfedges_list[current_idx]);
      current_idx = (current_idx + 1) % halfedges_list.size();
      if (path2.size() > halfedges_list.size()) {
        return nullptr;
      }
    }


    // 新しい face
    auto f_new = mesh_->addFace();

    // 新しい内部エッジ
    auto he1 = mesh_->addHalfedge(f, he_v1->vertex());     // v1→v2, f側
    auto he2 = mesh_->addHalfedge(f_new, he_v2->vertex()); // v2→v1, f_new側
    
    // --- ここから修正: mate/face/vertex参照を厳密に設定 ---
    // Set mate relationships for the new edge IMMEDIATELY
    he1->setMate(he2);
    he2->setMate(he1);
    
    // face参照を確実に設定
    he1->setFace(f);
    he2->setFace(f_new);
    
    // vertex参照を確実に設定
    he1->setVertex(he_v1->vertex());
    he2->setVertex(he_v2->vertex());
    // --- ここまで修正 ---
    

    // 新しい halfedge リスト（重複を除外）
    std::vector<std::shared_ptr<HalfedgeL>> f_halfedges = {he1};
    std::set<int> seenf;
    for (auto& h : path1) {
      if (h != he1 && h != he2 && h != he_v1 && h != he_v2 &&
          seenf.insert(h->id()).second) {
        f_halfedges.push_back(h);
      }
    }
    rebuildFaceHalfedges(f, f_halfedges);
    
    // --- ここから修正: face参照を再設定（強化版） ---
    for (auto he : f->halfedges()) {
      he->setFace(f);
    }
    // 新規halfedgeのface参照を確実に設定
    he1->setFace(f);
    // --- ここまで修正 ---
    
    
    // --- ここから追加: MEF後のエッジ状態デバッグ ---
    // --- ここまで追加 ---

    std::vector<std::shared_ptr<HalfedgeL>> f_new_halfedges = {he2};
    std::set<int> seenf_new;
    for (auto& h : path2) {
      if (h != he1 && h != he2 && h != he_v1 && h != he_v2 &&
          seenf_new.insert(h->id()).second) {
        f_new_halfedges.push_back(h);
      }
    }
    rebuildFaceHalfedges(f_new, f_new_halfedges);
    
    // --- ここから修正: face参照を再設定（強化版） ---
    for (auto he : f_new->halfedges()) {
      he->setFace(f_new);
    }
    // 新規halfedgeのface参照を確実に設定
    he2->setFace(f_new);
    // --- ここまで修正 ---
    
    
    // --- ここから追加: MEF後のエッジ状態デバッグ ---
    
    // 最終確認: エッジの両側のfaceが異なるかチェック
    if (he1->face() && he2->face()) {
    }
    // --- ここまで追加 ---

    // Update vertex halfedge pointers
    ensureVertexHalfedges(mesh_);
    
    // Verify mate relationship for the new edge (after ensureVertexHalfedges)
    if (he1->mate() != he2 || he2->mate() != he1) {
      // Try to fix the mate relationship
      he1->setMate(he2);
      he2->setMate(he1);
    } else {
    }
    
    
    return f_new;
  }

  // 3. Kill Edge Make Ring (KEMR) - Remove an edge and merge two faces
  bool killEdgeMakeRing(std::shared_ptr<HalfedgeL> he) {
    if (!mesh_ || !he) return false;

    auto mate = he->mate();
    if (!mate) return false;  // Boundary edge

    auto face1 = he->face();
    auto face2 = mate->face();
    if (!face1 || !face2) return false;

    // --- 修正された安全チェック ---
    // face1==face2の場合はself-loopエッジなので削除不可
    if (face1 == face2) {
      return false;
    }
    
    // エッジの状態を詳細にチェック
    
    // エッジが実際に異なるfaceに属しているか確認
    if (he->face() && mate->face() && he->face()->id() == mate->face()->id()) {
      return false;
    }
    
    // 基本的な参照チェック
    if (!he->vertex() || !he->next() || !he->next()->vertex()) {
      return false;
    }
    if (!mate->vertex() || !mate->next() || !mate->next()->vertex()) {
      return false;
    }
    
    // エッジが実際にメッシュに存在するかチェック
    bool he_found = false, mate_found = false;
    for (auto face : mesh_->faces()) {
      for (auto he_face : face->halfedges()) {
        if (he_face == he) he_found = true;
        if (he_face == mate) mate_found = true;
      }
    }
    if (!he_found || !mate_found) {
      return false;
    }
    
    // 削除対象エッジの隣接エッジが存在するかチェック
    auto he_prev = he->prev();
    auto he_next = he->next();
    auto mate_prev = mate->prev();
    auto mate_next = mate->next();
    
    if (!he_prev || !he_next || !mate_prev || !mate_next) {
      return false;
    }
    // --- ここまで修正された安全チェック ---

    // Get the vertices
    auto v1 = he->vertex();
    auto v2 = mate->vertex();

    // Collect all halfedges from face2 that will be merged into face1
    std::vector<std::shared_ptr<HalfedgeL>> face1_halfedges;
    std::vector<std::shared_ptr<HalfedgeL>> face2_halfedges;

    // Collect face1 halfedges (excluding he)
    auto current = he_next;
    int guard1 = 0;
    while (current != he && guard1++ < 100) {
      face1_halfedges.push_back(current);
      current = current->next();
    }

    // Collect face2 halfedges (excluding mate)
    current = mate_next;
    int guard2 = 0;
    while (current != mate && guard2++ < 100) {
      face2_halfedges.push_back(current);
      current = current->next();
    }


    // --- 修正された削除処理（順序を変更） ---
    
    // 1. 削除対象halfedgeとmateの参照をクリア（削除前に実行）
    he->setMate(nullptr);
    he->setFace(nullptr);
    he->setVertex(nullptr);
    mate->setMate(nullptr);
    mate->setFace(nullptr);
    mate->setVertex(nullptr);
    
    
    // 2. vertex halfedgeの更新（削除対象halfedgeを指している場合）
    if (v1->halfedge() == he) {
      updateVertexHalfedge(v1, he_next);
    }
    if (v2->halfedge() == mate) {
      updateVertexHalfedge(v2, mate_next);
    }
    
    
    // 3. face2のhalfedgeをface1に移動（face2削除前）
    for (auto he_face2 : face2_halfedges) {
      he_face2->setFace(face1);
    }
    
    
    // 4. face1のhalfedgeリストを統合されたhalfedgeで再構築
    std::vector<std::shared_ptr<HalfedgeL>> merged_halfedges;
    std::set<int> seen;
    for (auto& h : face1_halfedges) {
      if (seen.insert(h->id()).second) {
        merged_halfedges.push_back(h);
      }
    }
    for (auto& h : face2_halfedges) {
      if (seen.insert(h->id()).second) {
        merged_halfedges.push_back(h);
      }
    }
    
    
    rebuildFaceHalfedges(face1, merged_halfedges);
    
    // 5. face2を削除（halfedge削除前）
    mesh_->deleteFace(face2);
    
    // 6. meshからhalfedgeを削除（face2削除後）
    mesh_->deleteHalfedge(he);
    mesh_->deleteHalfedge(mate);
    
    // --- ここまで修正された削除処理 ---

    // --- 修正された後処理 ---
    // 7. mate関係を更新
    updateAllMates();
    
    // 8. すべてのvertex halfedgeを確実に更新
    ensureVertexHalfedges(mesh_);
    
    // 9. 最終的なmate関係の検証
    bool mate_relationships_valid = true;
    for (auto face : mesh_->faces()) {
      for (auto he_face : face->halfedges()) {
        if (he_face->mate() && he_face->mate()->mate() != he_face) {
          mate_relationships_valid = false;
        }
      }
    }
    
    if (!mate_relationships_valid) {
      updateAllMates();
    }
    
    // --- ここまで修正された後処理 ---

    return true;
  }

  // 4. Make Edge Kill Ring (MEKR) - Add an edge to split a face
  std::shared_ptr<HalfedgeL> makeEdgeKillRing(std::shared_ptr<VertexL> v1,
                                              std::shared_ptr<VertexL> v2,
                                              std::shared_ptr<FaceL> face) {
    if (!mesh_ || !v1 || !v2 || !face || v1 == v2) return nullptr;

    // Check if vertices are in the face
    bool v1_in_face = false, v2_in_face = false;
    std::shared_ptr<HalfedgeL> he_v1 = nullptr, he_v2 = nullptr;

    for (auto he : face->halfedges()) {
      if (he->vertex() == v1) {
        v1_in_face = true;
        he_v1 = he;
      }
      if (he->vertex() == v2) {
        v2_in_face = true;
        he_v2 = he;
      }
    }

    if (!v1_in_face || !v2_in_face || !he_v1 || !he_v2) return nullptr;

    // Create new face
    auto new_face = mesh_->addFace();

    // Create new halfedges
    auto he_new1 = mesh_->addHalfedge(face, v1);
    auto he_new2 = mesh_->addHalfedge(new_face, v2);

    // Set mate relationships
    setMate(he_new1, he_new2);

    // Find the path from v1 to v2 in the original face
    std::vector<std::shared_ptr<HalfedgeL>> path1, path2;
    auto current = he_v1;

    // Walk from v1 to v2
    do {
      path1.push_back(current);
      current = current->next();
    } while (current != he_v2);

    // Walk from v2 back to v1
    do {
      path2.push_back(current);
      current = current->next();
    } while (current != he_v1);

    // Update face assignments for path2
    for (auto he : path2) {
      he->setFace(new_face);
    }

    // Rebuild face halfedge lists
    std::vector<std::shared_ptr<HalfedgeL>> face1_halfedges = {he_new1};
    face1_halfedges.insert(face1_halfedges.end(), path1.begin(), path1.end());
    rebuildFaceHalfedges(face, face1_halfedges);

    std::vector<std::shared_ptr<HalfedgeL>> face2_halfedges = {he_new2};
    face2_halfedges.insert(face2_halfedges.end(), path2.begin(), path2.end());
    rebuildFaceHalfedges(new_face, face2_halfedges);

    updateAllMates();
    // Update vertex halfedges if needed
    ensureVertexHalfedges(mesh_);

    return he_new1;
  }

  // 5. Kill Face Make Ring Hole (KFMRH) - Remove a face to create a hole
  bool killFaceMakeRingHole(std::shared_ptr<FaceL> face) {
    if (!mesh_ || !face) return false;


    // Check if face is a simple polygon (no holes)
    auto halfedges = face->halfedges();
    if (halfedges.empty()) return false;

    // Update vertex halfedges if they point to this face
    for (auto he : halfedges) {
      auto vertex = he->vertex();
      if (vertex->halfedge() == he) {
        // Find another halfedge for this vertex
        auto next_he = he->next();
        if (next_he != he) {
          updateVertexHalfedge(vertex, next_he);
        }
      }
    }

    // Remove the face
    mesh_->deleteFace(face);

    updateAllMates();
    // Ensure all vertices have halfedge pointers
    ensureVertexHalfedges(mesh_);


    return true;
  }

  // 6. Make Face Kill Ring Hole (MFKRH) - Fill a hole with a new face
  std::shared_ptr<FaceL> makeFaceKillRingHole(
      std::vector<std::shared_ptr<VertexL>> vertices) {
    if (!mesh_ || vertices.size() < 3) return nullptr;

    // Create new face
    auto face = mesh_->addFace();

    // Create halfedges for the face
    std::vector<std::shared_ptr<HalfedgeL>> halfedges;
    std::set<int> seen;
    for (size_t i = 0; i < vertices.size(); ++i) {
      auto he = mesh_->addHalfedge(face, vertices[i]);
      if (seen.insert(he->id()).second) halfedges.push_back(he);
    }

    // Rebuild face's halfedge list
    rebuildFaceHalfedges(face, halfedges);

    updateAllMates();
    // Update vertex halfedges if needed
    for (auto he : halfedges) {
      auto vertex = he->vertex();
      if (!vertex->halfedge()) {
        updateVertexHalfedge(vertex, he);
      }
    }

    // Ensure all vertices have halfedge pointers
    ensureVertexHalfedges(mesh_);

    return face;
  }

  // ============================================================================
  // Inverse Euler Operations
  // ============================================================================

  // Kill Edge Vertex (KEV) - Remove a vertex by merging its incident edges
  bool killEdgeVertex(std::shared_ptr<VertexL> vertex) {
    if (!mesh_ || !vertex) return false;

    // Find a halfedge that points to this vertex
    std::shared_ptr<HalfedgeL> he = nullptr;
    for (auto face : mesh_->faces()) {
      for (auto he_face : face->halfedges()) {
        if (he_face->vertex() == vertex) {
          he = he_face;
          break;
        }
      }
      if (he) break;
    }

    if (!he) {
      return false;
    }

    auto mate = he->mate();
    if (!mate) {
      return false;
    }

    // Get the vertices
    auto v1 = he->vertex();
    auto v2 = mate->vertex();
    auto v3 = he->next()->vertex();

    // Get the faces
    auto face1 = he->face();
    auto face2 = mate->face();

    // Get the halfedges around the vertex
    auto he_prev = he->prev();
    auto he_next = he->next();
    auto mate_prev = mate->prev();
    auto mate_next = mate->next();

    // Collect halfedges from both faces
    std::vector<std::shared_ptr<HalfedgeL>> face1_halfedges;
    std::vector<std::shared_ptr<HalfedgeL>> face2_halfedges;

    // Collect face1 halfedges (excluding he)
    auto current = he_next;
    int guard1 = 0;
    while (current != he && guard1++ < 100) {
      face1_halfedges.push_back(current);
      current = current->next();
    }

    // Collect face2 halfedges (excluding mate)
    current = mate_next;
    int guard2 = 0;
    while (current != mate && guard2++ < 100) {
      face2_halfedges.push_back(current);
      current = current->next();
    }

    // --- ここから修正: 削除対象halfedgeとmateの参照をクリア ---
    // 削除対象halfedgeとmateの参照をクリア
    he->setMate(nullptr);
    he->setFace(nullptr);
    he->setVertex(nullptr);
    mate->setMate(nullptr);
    mate->setFace(nullptr);
    mate->setVertex(nullptr);
    
    // 削除前に、このhalfedgeを参照している他の要素の参照もクリア
    for (auto fc : mesh_->faces()) {
      for (auto he_face : fc->halfedges()) {
        if (he_face == he || he_face == mate) {
          he_face->setMate(nullptr);
          he_face->setFace(nullptr);
          he_face->setVertex(nullptr);
        }
      }
    }
    
    // meshからhalfedgeを削除
    mesh_->deleteHalfedge(he);
    mesh_->deleteHalfedge(mate);
    // --- ここまで修正 ---

    // Update all halfedges in face2 to point to face1
    for (auto he_face2 : face2_halfedges) {
      he_face2->setFace(face1);
    }

    // Rebuild face1's halfedge list with merged halfedges
    std::vector<std::shared_ptr<HalfedgeL>> merged_halfedges;
    std::set<int> seen;
    for (auto& h : face1_halfedges) if (seen.insert(h->id()).second) merged_halfedges.push_back(h);
    for (auto& h : face2_halfedges) if (seen.insert(h->id()).second) merged_halfedges.push_back(h);
    rebuildFaceHalfedges(face1, merged_halfedges);

    updateAllMates();
    // Update vertex halfedges if needed
    if (v1->halfedge() == he) updateVertexHalfedge(v1, he_next);
    if (v2->halfedge() == mate) updateVertexHalfedge(v2, mate_next);

    // Remove the old face
    mesh_->deleteFace(face2);

    // Remove the vertex
    mesh_->deleteVertex(vertex);

    // --- ここから修正: 削除後の完全なリスト再構築 ---
    // 削除後にface/vertex/halfedgeリストを完全に再構築
    for (auto fc : mesh_->faces()) {
      std::vector<std::shared_ptr<HalfedgeL>> valid_halfedges;
      for (auto he_face : fc->halfedges()) {
        if (he_face && he_face->vertex() && he_face->face()) {
          valid_halfedges.push_back(he_face);
        }
      }
      fc->deleteHalfedges();
      for (auto he_face : valid_halfedges) {
        fc->addHalfedge(he_face);
      }
    }
    // --- ここまで修正 ---

    // Ensure all vertices have halfedge pointers
    ensureVertexHalfedges(mesh_);

    return true;
  }

  // KEV: Kill Edge and Vertex (OpenMesh-style)
  static bool killEdgeVertexOpenMesh(std::shared_ptr<MeshL> mesh, std::shared_ptr<VertexL> v) {
    // 前提条件チェック: vが2-valentで、2つのhalfedgeがmate関係で内部エッジを形成していること
    if (!mesh || !v) return false;
    std::vector<std::shared_ptr<HalfedgeL>> incoming;
    for (auto face : mesh->faces()) {
      for (auto he : face->halfedges()) {
        if (he->next()->vertex() == v) incoming.push_back(he);
      }
    }
    if (incoming.size() != 2) {
      return false;
    }
    auto he1 = incoming[0];
    auto he2 = incoming[1];
    if (!he1->mate() || he1->mate() != he2) {
      return false;
    }
    // mate/face整合性: he1,he2のfaceが異なること
    if (he1->face() == he2->face()) {
      return false;
    }
    
    // --- 削除前に他要素の参照をクリア ---
    he1->setMate(nullptr);
    he2->setMate(nullptr);
    
    // faceからhalfedgeを除外（削除前にリストを保存）
    auto face1 = he1->face();
    auto face2 = he2->face();
    
    // 削除前にhalfedgeリストを保存
    std::vector<std::shared_ptr<HalfedgeL>> face1_halfedges, face2_halfedges;
    for (auto he : face1->halfedges()) {
      if (he != he1 && he->vertex() != v) {
        face1_halfedges.push_back(he);
      }
    }
    for (auto he : face2->halfedges()) {
      if (he != he2 && he->vertex() != v) {
        face2_halfedges.push_back(he);
      }
    }
    
    // faceのhalfedgeリストをクリアして再構築
    face1->deleteHalfedges();
    face2->deleteHalfedges();
    
    EulerOperations temp_euler(mesh);
    temp_euler.rebuildFaceHalfedges(face1, face1_halfedges);
    temp_euler.rebuildFaceHalfedges(face2, face2_halfedges);
    
    // vのhalfedge参照をクリア
    v->setHalfedge(nullptr);
    
    // meshからvを削除
    mesh->deleteVertex(v);
    
    // 頂点のhalfedgeポインタを更新
    temp_euler.ensureVertexHalfedges(mesh);
    
    return true;
  }

  // KEF: Kill Edge and Face (OpenMesh-style)
  static bool killEdgeFaceOpenMesh(std::shared_ptr<MeshL> mesh, std::shared_ptr<VertexL> v1, std::shared_ptr<VertexL> v2) {
    // 前提条件チェック: v1-v2間のエッジが内部エッジで、2つのfaceにまたがっていること
    if (!mesh || !v1 || !v2) return false;
    std::shared_ptr<HalfedgeL> he1 = nullptr, he2 = nullptr;
    for (auto face : mesh->faces()) {
      for (auto he : face->halfedges()) {
        if (he->vertex() == v1 && he->mate() && he->mate()->vertex() == v2) {
          he1 = he;
          he2 = he->mate();
          break;
        }
      }
      if (he1 && he2) break;
    }
    if (!he1 || !he2) {
      return false;
    }
    if (!he1->mate() || he1->mate() != he2) {
      return false;
    }
    if (he1->face() == he2->face()) {
      return false;
    }
    // --- 削除前に他要素の参照をクリア ---
    he1->setMate(nullptr);
    he2->setMate(nullptr);
    
    // face2を削除、face1にhalfedgeを統合
    auto face1 = he1->face();
    auto face2 = he2->face();
    
    // 削除前にhalfedgeリストを保存
    std::vector<std::shared_ptr<HalfedgeL>> merged_halfedges;
    for (auto he : face1->halfedges()) {
      if (he != he1) {
        merged_halfedges.push_back(he);
      }
    }
    for (auto he : face2->halfedges()) {
      if (he != he2) {
        merged_halfedges.push_back(he);
      }
    }
    
    // face1のhalfedgeリストをクリアして再構築
    face1->deleteHalfedges();
    
    EulerOperations temp_euler(mesh);
    temp_euler.rebuildFaceHalfedges(face1, merged_halfedges);
    
    // meshからface2を削除
    mesh->deleteFace(face2);
    
    // 頂点のhalfedgeポインタを更新
    temp_euler.ensureVertexHalfedges(mesh);
    
    return true;
  }

  // ============================================================================
  // Advanced Operations
  // ============================================================================

  // Create a triangle face
  std::shared_ptr<FaceL> createTriangle(std::shared_ptr<VertexL> v1,
                                        std::shared_ptr<VertexL> v2,
                                        std::shared_ptr<VertexL> v3) {
    if (!mesh_ || !v1 || !v2 || !v3) return nullptr;

    // Create new face
    auto face = mesh_->addFace();

    // Create halfedges
    auto he1 = mesh_->addHalfedge(face, v1);
    auto he2 = mesh_->addHalfedge(face, v2);
    auto he3 = mesh_->addHalfedge(face, v3);

    // Rebuild face's halfedge list
    std::vector<std::shared_ptr<HalfedgeL>> halfedges = {he1, he2, he3};
    rebuildFaceHalfedges(face, halfedges);

    // Update vertex halfedges if needed
    if (!v1->halfedge()) updateVertexHalfedge(v1, he1);
    if (!v2->halfedge()) updateVertexHalfedge(v2, he2);
    if (!v3->halfedge()) updateVertexHalfedge(v3, he3);

    return face;
  }

  // Create a quad face
  std::shared_ptr<FaceL> createQuad(std::shared_ptr<VertexL> v1,
                                    std::shared_ptr<VertexL> v2,
                                    std::shared_ptr<VertexL> v3,
                                    std::shared_ptr<VertexL> v4) {
    if (!mesh_ || !v1 || !v2 || !v3 || !v4) return nullptr;

    // Create new face
    auto face = mesh_->addFace();

    // Create halfedges
    auto he1 = mesh_->addHalfedge(face, v1);
    auto he2 = mesh_->addHalfedge(face, v2);
    auto he3 = mesh_->addHalfedge(face, v3);
    auto he4 = mesh_->addHalfedge(face, v4);

    // Rebuild face's halfedge list
    std::vector<std::shared_ptr<HalfedgeL>> halfedges = {he1, he2, he3, he4};
    rebuildFaceHalfedges(face, halfedges);

    // Update vertex halfedges if needed
    if (!v1->halfedge()) updateVertexHalfedge(v1, he1);
    if (!v2->halfedge()) updateVertexHalfedge(v2, he2);
    if (!v3->halfedge()) updateVertexHalfedge(v3, he3);
    if (!v4->halfedge()) updateVertexHalfedge(v4, he4);

    return face;
  }

  // ============================================================================
  // Validation and Debugging
  // ============================================================================

  // Validate mesh connectivity
  bool validateMesh() const {
    if (!mesh_) return false;

    // Simple validation: check if mesh has basic structure
    if (mesh_->vertices_size() == 0) {
      return false;
    }

    if (mesh_->faces_size() == 0) {
      return false;
    }

    // Check if all faces have halfedges
    for (auto face : mesh_->faces()) {
      if (face->halfedges().empty()) {
        return false;
      }
    }

    // 最終的なEuler characteristic計算
    int V = mesh_->vertices_size();
    int E = countUniqueEdges();
    int F = mesh_->faces_size();
    int euler_char = V - E + F;

    // Debug: print face details
    for (auto face : mesh_->faces()) {
      for (auto he : face->halfedges()) {
      }
    }

    if (euler_char == 2) {
    } else if (euler_char == 1) {
    } else if (euler_char == 0) {
    } else {
      // 詳細デバッグ出力
      for (auto v : mesh_->vertices()) {
      }
      for (auto f : mesh_->faces()) {
        for (auto he : f->halfedges()) {
        }
      }
      for (auto f : mesh_->faces()) {
        for (auto he : f->halfedges()) {
        }
      }
    }

    // 参照整合性チェック
    bool valid = true;
    std::set<int> valid_vertex_ids, valid_face_ids;
    for (auto v : mesh_->vertices()) valid_vertex_ids.insert(v->id());
    for (auto f : mesh_->faces()) valid_face_ids.insert(f->id());

    // 頂点のhalfedge参照チェック
    for (auto v : mesh_->vertices()) {
      auto he = v->halfedge();
      if (he) {
        if (!he->vertex() || valid_vertex_ids.find(he->vertex()->id()) == valid_vertex_ids.end()) {
          valid = false;
        }
        if (!he->face() || valid_face_ids.find(he->face()->id()) == valid_face_ids.end()) {
          valid = false;
        }
      }
    }
    // faceのhalfedgeリスト参照チェック
    for (auto f : mesh_->faces()) {
      for (auto he : f->halfedges()) {
        if (!he->vertex() || valid_vertex_ids.find(he->vertex()->id()) == valid_vertex_ids.end()) {
          valid = false;
        }
        if (!he->face() || valid_face_ids.find(he->face()->id()) == valid_face_ids.end()) {
          valid = false;
        }
      }
    }
    // mate参照チェック
    for (auto f : mesh_->faces()) {
      for (auto he : f->halfedges()) {
        if (he->mate()) {
          if (!he->mate()->vertex() || valid_vertex_ids.find(he->mate()->vertex()->id()) == valid_vertex_ids.end()) {
            valid = false;
          }
          if (!he->mate()->face() || valid_face_ids.find(he->mate()->face()->id()) == valid_face_ids.end()) {
            valid = false;
          }
        }
      }
    }
    if (!valid) {
    }

    return (euler_char == 2 || euler_char == 1 || euler_char == 0);
  }

  // Count unique edges in the mesh (corrected for boundary edges)
  int countUniqueEdges() const {
    if (!mesh_) return 0;
    std::set<std::pair<int, int>> edge_set;
    int boundary_edges = 0;
    int internal_edges = 0;
    
    // --- ここから修正: より正確なエッジカウントロジック ---
    // 全てのhalfedgeを走査してエッジをカウント
    std::set<int> processed_halfedges;
    
    for (auto face : mesh_->faces()) {
      if (!face) continue;
      
      for (auto he : face->halfedges()) {
        if (!he || !he->vertex() || !he->next() || !he->next()->vertex()) {
          continue;
        }
        
        // 既に処理済みのhalfedgeはスキップ
        if (processed_halfedges.find(he->id()) != processed_halfedges.end()) {
          continue;
        }
        
        int v1 = he->vertex()->id();
        int v2 = he->next()->vertex()->id();
        
        // 常に小さいIDをv1、大きいIDをv2にする（重複カウント防止）
        if (v1 > v2) std::swap(v1, v2);
        
        auto edge_pair = std::make_pair(v1, v2);
        if (edge_set.find(edge_pair) == edge_set.end()) {
          edge_set.insert(edge_pair);
          
          // このhalfedgeとそのmateを処理済みとしてマーク
          processed_halfedges.insert(he->id());
          if (he->mate()) {
            processed_halfedges.insert(he->mate()->id());
          }
          
          // Check if this is a boundary edge (no mate) or internal edge (has mate)
          if (he->mate() && he->mate()->face() && he->face() && 
              he->mate()->face()->id() != he->face()->id()) {
            internal_edges++;
          } else {
            boundary_edges++;
          }
        }
      }
    }
    // --- ここまで修正 ---
    
    
    return internal_edges + boundary_edges;
  }

  // ============================================================================
  // Utility functions
  // ============================================================================

  void printMeshInfo() const {
    if (!mesh_) return;


    int V = mesh_->vertices_size();
    int F = mesh_->faces_size();
    int E = countUniqueEdges();
    int chi = V - E + F;

    int vertices_with_halfedge = 0;
    for (auto vertex : mesh_->vertices()) {
      if (vertex->halfedge()) vertices_with_halfedge++;
    }

    // --- 詳細出力 ---
    for (auto v : mesh_->vertices()) {
    }
    for (auto f : mesh_->faces()) {
      for (auto he : f->halfedges()) {
      }
      for (auto he : f->halfedges()) {
      }
    }
    for (auto f : mesh_->faces()) {
      for (auto he : f->halfedges()) {
      }
    }
  }

  void printFaceInfo(std::shared_ptr<FaceL> face) const {
    if (!face) return;

    int i = 0;
    for (auto he : face->halfedges()) {
    }
  }

  void printVertexInfo(std::shared_ptr<VertexL> vertex) const {
    if (!vertex) return;

    if (vertex->halfedge()) {
    }
  }

  // Helper function to create a face with full vertex connectivity
  std::shared_ptr<FaceL> createFaceWithFullConnectivity(
      const std::vector<std::shared_ptr<VertexL>>& vertices) {
    if (!mesh_ || vertices.size() < 3) return nullptr;

    auto face = mesh_->addFace();
    std::vector<std::shared_ptr<HalfedgeL>> halfedges;

    // Create halfedges
    for (auto vertex : vertices) {
      auto he = mesh_->addHalfedge(face, vertex);
      halfedges.push_back(he);
    }

    // Rebuild face's halfedge list
    rebuildFaceHalfedges(face, halfedges);

    // Ensure all vertices have halfedge pointers
    for (size_t i = 0; i < vertices.size(); ++i) {
      if (!vertices[i]->halfedge()) {
        vertices[i]->setHalfedge(halfedges[i]);
      }
    }

    return face;
  }

  // 安全にKEVできるか判定
  bool canKillEdgeVertex(std::shared_ptr<VertexL> v) const {
    if (!v || !v->halfedge()) return false;
    auto he = v->halfedge();
    std::set<int> visited;
    do {
      if (!he->mate()) return false; // 境界
      if (!he->next()) return false;
      if (visited.count(he->id())) break;
      visited.insert(he->id());
      he = he->mate()->next();
    } while (he && he != v->halfedge());
    return true;
  }

  // 安全にKEFできるか判定（修正版）
  bool canKillEdgeMakeRing(std::shared_ptr<HalfedgeL> he) const {
    
    if (!he) {
      return false;
    }
    
    if (!he->mate()) {
      return false;
    }
    
    auto f1 = he->face();
    auto f2 = he->mate()->face();
    
    if (!f1 || !f2) {
      return false;
    }
    
    if (f1 == f2) {
      return false; // self-loop
    }
    
    // Additional safety checks
    if (!he->vertex() || !he->next() || !he->next()->vertex()) {
      return false;
    }
    
    if (!he->mate()->vertex() || !he->mate()->next() || !he->mate()->next()->vertex()) {
      return false;
    }
    
    // Check if the edge is still valid in the mesh
    bool he_found = false, mate_found = false;
    for (auto face : mesh_->faces()) {
      for (auto he_face : face->halfedges()) {
        if (he_face == he) he_found = true;
        if (he_face == he->mate()) mate_found = true;
      }
    }
    
    if (!he_found || !mate_found) {
      return false;
    }
    
    // Additional check: ensure the edge is not a boundary edge
    if (!he->mate() || !he->mate()->face()) {
      return false;
    }
    
    // Check if both faces have more than 3 edges (to avoid creating degenerate faces)
    int face1_edges = 0, face2_edges = 0;
    for (auto he_face : f1->halfedges()) face1_edges++;
    for (auto he_face : f2->halfedges()) face2_edges++;
    
    if (face1_edges <= 3 || face2_edges <= 3) {
      return false;
    }
    
    return true;
  }

 private:
  std::shared_ptr<MeshL> mesh_;
};

#endif  // _EULEROPERATIONS_HXX
