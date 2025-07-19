////////////////////////////////////////////////////////////////////
//
// $Id: MeshL.hxx 2025/07/17 23:56:43 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _MESHL_HXX
#define _MESHL_HXX 1

#include "envDep.h"

#include <list>
#include <vector>
#include <memory>
//using namespace std;

#include "myEigen.hxx"

#include "VertexL.hxx"
#include "mydef.h"
#include "NormalL.hxx"
#include "TexcoordL.hxx"
#include "HalfedgeL.hxx"
#include "FaceL.hxx"
#include "EdgeL.hxx"
#include "LoopL.hxx"
#include "BLoopL.hxx"
#include "VertexLCirculator.hxx"
#include "MeshUtiL.hxx"

class MeshL {

 public:

  MeshL() { init(); };
  ~MeshL() { deleteAll(); };

  void init() {
    v_id_ = 0;
    f_id_ = 0;
    h_id_ = 0;
    t_id_ = 0;
    e_id_ = 0;
    n_id_ = 0;
    bl_id_ = 0;
    l_id_ = 0;
    isConnectivity_ = false;
    isNormalized_ = false;
    texID_ = 0;
  };

  // elements
  std::list<std::shared_ptr<VertexL> >& vertices() { return vertices_; };
  int vertices_size() const { return (int)vertices_.size(); };
  std::list<std::shared_ptr<NormalL> >& normals() { return normals_; };
  std::list<std::shared_ptr<TexcoordL> >& texcoords() { return texcoords_; };
  std::list<std::shared_ptr<FaceL> >& faces() { return faces_; };
  int faces_size() const { return (int)faces_.size(); };
  std::list<std::shared_ptr<EdgeL> >& edges() { return edges_; };
  std::list<std::shared_ptr<HalfedgeL> >& halfedges() { return halfedges_; }
  const std::list<std::shared_ptr<HalfedgeL> >& halfedges() const { return halfedges_; }
  int halfedges_size() const { return (int)halfedges_.size(); };
  std::list<std::shared_ptr<LoopL> >& loops() { return loops_; };
  std::list<std::shared_ptr<BLoopL> >& bloops() { return bloops_; };
  //    Material& material() { return material_; };

  // vertex
  std::shared_ptr<VertexL> vertex(int id) {
    for (auto& vt : vertices_)
      if (vt->id() == id) return vt;
    return nullptr;
  };

  std::shared_ptr<VertexL> addVertex(Eigen::Vector3d& p) {
    std::shared_ptr<VertexL> vt = addVertex();
    vt->setPoint(p);
    return vt;
  };

  void deleteVertex(std::shared_ptr<VertexL> vt) {
    vertices_.erase(vt->iter());
    //delete vt;
  };

  // normal
  std::shared_ptr<NormalL> normal(int id) {
    for (auto& nm : normals_)
      if (nm->id() == id) return nm;
    return nullptr;
  };

  std::shared_ptr<NormalL> addNormal(Eigen::Vector3d& p) {
    std::shared_ptr<NormalL> nm = addNormal();
    nm->setPoint(p);
    return nm;
  };

  void deleteNormal(std::shared_ptr<NormalL> nm) {
    normals_.erase(nm->iter());
    //delete nm;
  }

  // texcoord
  std::shared_ptr<TexcoordL> texcoord(int id) {
    for (auto& tc : texcoords_)
      if (tc->id() == id) return tc;
    return nullptr;
  };

  std::shared_ptr<TexcoordL> addTexcoord(Eigen::Vector3d& p) {
    std::shared_ptr<TexcoordL> tc = addTexcoord();
    tc->setPoint(p);
    return tc;
  };

  void deleteTexcoord(std::shared_ptr<TexcoordL> tc) {
    texcoords_.erase(tc->iter());
    //delete tc;
  };

  // halfedge
  std::shared_ptr<HalfedgeL> halfedge(int id) {
    for (auto& he : halfedges_)
      if (he->id() == id) return he;
    return nullptr;
  };

  std::shared_ptr<HalfedgeL> addHalfedge(std::shared_ptr<FaceL> fc) {
    std::shared_ptr<HalfedgeL> he = addHalfedge();
    fc->addHalfedge(he);
    return he;
  };

  std::shared_ptr<HalfedgeL> addHalfedge(std::shared_ptr<FaceL> fc,
                                         std::shared_ptr<VertexL> vt,
                                         std::shared_ptr<NormalL> nm = nullptr,
                                         std::shared_ptr<TexcoordL> tc = nullptr) {
    std::shared_ptr<HalfedgeL> he = addHalfedge();
    fc->addHalfedge(he, vt, nm, tc);
    return he;
  };

  // he の後にハーフエッジを挿入
  std::shared_ptr<HalfedgeL> insertHalfedge(std::shared_ptr<FaceL> fc,
                                            std::shared_ptr<HalfedgeL> he,
                                            std::shared_ptr<VertexL> vt,
                                            std::shared_ptr<NormalL> nm = nullptr,
                                            std::shared_ptr<TexcoordL> tc = nullptr) {
    std::shared_ptr<HalfedgeL> nhe = addHalfedge();
    fc->insertHalfedge(nhe, he, vt, nm, tc);
    return nhe;
  };

  // 不要なhalfedgeを削除
  void deleteHalfedge(std::shared_ptr<HalfedgeL> he) {
    if (!he) return;
    
    // 削除前に参照をクリア
    if (he->vertex() && he->vertex()->halfedge() == he) {
      he->vertex()->setHalfedge(nullptr);
    }
    
    // 削除前に、このhalfedgeを参照している他のhalfedgeのmate参照をクリア
    if (he->mate()) {
      he->mate()->setMate(nullptr);
    }
    
    // faceリストから削除（faceが存在する場合のみ）
    std::shared_ptr<FaceL> fc = he->face();
    if (fc && he->f_iter() != fc->halfedges().end()) {
      //std::cout << "delete face lists"<< std::endl;
      fc->halfedges().erase(he->f_iter());
    }
    
    // 全体リストからも削除（イテレータが有効な場合のみ）
    if (he->meshIter() != halfedges_.end()) {
      halfedges_.erase(he->meshIter());
    }

    he->setFace(nullptr);
    he->setMate(nullptr);
    he->setEdge(nullptr);
    he->setNormal(nullptr);
    he->setTexcoord(nullptr);
  };

  // face
  std::shared_ptr<FaceL> face(int id) {
    for (auto& fc : faces_)
      if (fc->id() == id) return fc;
    return nullptr;
  };

  std::shared_ptr<FaceL> addFace() {
    int32_t id = f_id_++;
    std::shared_ptr<FaceL> fc = std::make_shared<FaceL>(id);
    fc->setIter(faces_.insert(faces_.end(), fc));
    return fc;
  };
  
  void deleteFace(std::shared_ptr<FaceL> fc) {
    if (fc == nullptr) return;
    fc->deleteHalfedges();
    faces_.erase(fc->iter());
    //delete fc;
  };

  std::shared_ptr<FaceL> addTriangle( std::shared_ptr<VertexL> v0,
                                      std::shared_ptr<VertexL> v1,
                                      std::shared_ptr<VertexL> v2 ) {
    std::shared_ptr<FaceL> fc = addFace();
    addHalfedge(fc, v0);
    addHalfedge(fc, v1);
    addHalfedge(fc, v2);
    return fc;
  };

  // edge
  std::shared_ptr<EdgeL> addEdge(std::shared_ptr<VertexL> sv, std::shared_ptr<VertexL> ev) {
    std::shared_ptr<EdgeL> ed = addEdge();
    ed->setSVertex(sv);
    ed->setEVertex(ev);
    return ed;
  };

  void deleteEdge(std::shared_ptr<EdgeL> ed) {
    edges_.erase(ed->iter());
  }

  // loop
  std::shared_ptr<LoopL> addLoop() {
    int32_t id = l_id_++;
    std::shared_ptr<LoopL> lp = std::make_shared<LoopL>(id);
    lp->setIter(loops_.insert(loops_.end(), lp));
    return lp;
  }

  void deleteLoop(std::shared_ptr<LoopL> lp) {
    loops_.erase(lp->iter());
  }

  // boundary loop
  std::shared_ptr<BLoopL> addBLoop() {
    int32_t id = bl_id_++;
    std::shared_ptr<BLoopL> blp = std::make_shared<BLoopL>(id);
    blp->setBIter(bloops_.insert(bloops_.end(), blp));
    return blp;
  }

  void deleteBLoop(std::shared_ptr<BLoopL> blp) {
    bloops_.erase(blp->biter());
  }

  bool emptyBLoop() const { return bloops_.empty(); }
  std::shared_ptr<BLoopL> bloop() { return bloops_.empty() ? nullptr : *(bloops_.begin()); }

  // delete all
  void deleteAllVertices() {
    vertices_.clear();
  };

  void deleteAllNormals() {
    normals_.clear();
  };

  void deleteAllTexcoords() {
    texcoords_.clear();
  };

  void deleteAllHalfedges() {
    halfedges_.clear();
  };

  void deleteAllFaces() {
    faces_.clear();
  };

  void deleteAllEdges() {
    for (auto& ed : edges_) {
          if (ed->lhe() != nullptr) ed->lhe()->setEdge(nullptr);
    if (ed->rhe() != nullptr) ed->rhe()->setEdge(nullptr);
    }
    edges_.clear();
    e_id_ = 0;
  };

  void deleteAllLoops() {
    loops_.clear();
  };

  void deleteAllBLoops() {
    bloops_.clear();
  };

  void deleteAll() {
    deleteAllBLoops();
    deleteAllLoops();
    deleteAllEdges();
    deleteAllHalfedges();
    deleteAllFaces();
    deleteAllTexcoords();
    deleteAllNormals();
    deleteAllVertices();
  };

  // find
  std::shared_ptr<HalfedgeL> findHalfedge(std::shared_ptr<VertexL> sv, std::shared_ptr<VertexL> ev) {
    for (auto& fc : faces_) {
      for (auto& he : fc->halfedges()) {
        if ((he->vertex() == sv) && (he->next()->vertex() == ev)) return he;
        if ((he->vertex() == ev) && (he->next()->vertex() == sv)) return he;
      }
    }
    return nullptr;
  };

  std::shared_ptr<HalfedgeL> findDirectedHalfedge(std::shared_ptr<VertexL> sv, std::shared_ptr<VertexL> ev) {
    for (auto& he : halfedges_) {
      if ((he->vertex() == sv) && (he->next()->vertex() == ev)) return he;
    }
    return nullptr;
  };
      
  std::shared_ptr<VertexL> findSelectedVertex() {
    for (auto& vt : vertices_) {
      if (vt->isSelected() == true) return vt;
    }
    return nullptr;
  }

  std::shared_ptr<VertexL> findBoundaryVertex() {
    for (auto& vt : vertices_) {
      if (isBoundary(vt) == true) return vt;
    }
    return nullptr;
  }

  // center, max length for normalization
  void setCenter(Eigen::Vector3d& cen) { center_ = cen; };
  Eigen::Vector3d& center() { return center_; };
  void setMaxLength(double maxlen) { max_length_ = maxlen; };
  double maxLength() const { return max_length_; };
  bool isNormalized() const { return isNormalized_; };
  void setIsNormalized(bool f) { isNormalized_ = f; };

  // create
  void createConnectivity(bool isDeleteEdges=true) {
    // already defined
    if (isConnectivity()) {
      if (!(edges_.empty())) deleteAllEdges();
      deleteConnectivity();
    }

    std::vector<EdgeList> edge_list(vertices_size());
    for (auto& fc : faces_) {
      for (auto& he : fc->halfedges()) {
        auto sv = he->vertex();
        auto ev = he->next()->vertex();

        // set a halfedge to sv
        sv->setHalfedge(he);

        // find an edge
        std::shared_ptr<EdgeL> ed = nullptr;
        if ((ed = edge_list[sv->id()].findEdge(sv, ev)) == nullptr) {
          // new edge
          ed = addEdge(sv, ev);
          ed->setLHalfedge(he);
          edge_list[ed->sv()->id()].push_back(ed);
          edge_list[ed->ev()->id()].push_back(ed);
        } else {
          if (ed->rhe() != nullptr) {
            std::cerr << "Warning: More than three halfedges. Edge No."
                      << ed->id() << std::endl;
            std::cerr << "fc " << fc->id() << " he " << he->id()
                      << std::endl;
          }

          // assign right halfedge
          std::shared_ptr<HalfedgeL> lhe = ed->lhe();
          if (lhe->mate_valid(he)) {
            lhe->setMate(he);
            he->setMate(lhe);
          } else {
            std::cerr << "Warning: invalid halfedge pair. Edge No." << ed->id()
                      << std::endl;
          }

          if (ed->rhe_valid(he)) {
            ed->setRHalfedge(he);
          }
        }
      }
    }

    // move vt's halfedge to the end. (efficient for boundary vertex)
    for (auto& vt : vertices_) {
      auto he = vt->halfedge();
      if (he != nullptr) vt->setHalfedge(he->reset());
    }

    if (isDeleteEdges) deleteAllEdges();

    setConnectivity(true);
  };

  void deleteConnectivity() {
    for (auto& vt : vertices_) {
      vt->setHalfedge(nullptr);
    }

    for (auto& fc : faces_) {
      for (auto& he : fc->halfedges()) {
        he->setMate(nullptr);
      }
    }

    setConnectivity(false);
  }

  void checkConnectivity() {
    // face connectivity
    deleteIsolateFaces();
    // vertex in face check
    deleteIsolateVertices();
  };

  // delete isoleted faces
  void deleteIsolateFaces() {
    unsigned int i;
    std::vector<std::shared_ptr<FaceL>> fcount;
    for (auto& fc : faces_) {
      int count = 0;
      for (auto& he : fc->halfedges()) {
        if (he->isBoundary()) count++;
      }
      if (count >= 2) {
        std::cout << "Warning: Face No." << fc->id()
                  << " is isolated. deleted..." << std::endl;
        // 削除前にこのfaceを参照するhalfedgeのface参照をクリア
        for (auto& he : fc->halfedges()) {
          he->setFace(nullptr);
        }
        fcount.push_back(fc);
      }
    }
    for (i = 0; i < fcount.size(); ++i) deleteFace(fcount[i]);
  }

  // delete unused vertices
  void deleteIsolateVertices() {
    std::vector<int> vcount;
    vcount.resize(vertices_size());

    unsigned int v_size = vertices_size();
    unsigned int i;
    for (i = 0; i < v_size; ++i) vcount[i] = 0;

    for (auto& fc : faces_) {
      for (auto& he : fc->halfedges()) {
        vcount[he->vertex()->id()]++;
      }
    }

    bool isRecalculateBLoop = false;
    for (i = 0; i < v_size; ++i)
      if (!(vcount[i])) {
        std::cout << "Warning: Vertex No." << i
                  << " was not used in all faces. deleted..." << std::endl;
        auto vt = vertex(i);
        // 削除前にこのvertexを参照するhalfedgeのvertex参照をクリア
        for (auto& fc : faces_) {
          for (auto& he : fc->halfedges()) {
            if (he->vertex() == vt) {
              he->setVertex(nullptr);
            }
          }
        }
        // vertex自身のhalfedge参照もクリア
        vt->setHalfedge(nullptr);
        if (!emptyBLoop()) {
          if (bloop()->isVertex(vt) == true) isRecalculateBLoop = true;
        }
        deleteVertex(vt);
      }

    if (isRecalculateBLoop) createBLoop(bloop()->vertex(0));
    
    // 削除後にdangling halfedgeを一掃
    cleanupDanglingHalfedges();
    
    // 全faceのhalfedgeリストを再構築
    rebuildAllFaceHalfedgeLists();
  }

  // 削除済みvertex/faceを参照しているhalfedgeを一掃
  void cleanupDanglingHalfedges() {
    std::vector<std::shared_ptr<HalfedgeL>> to_delete;
    for (auto& he : halfedges_) {
      if (!he->vertex() || !he->face()) {
        to_delete.push_back(he);
      }
    }
    for (auto& he : to_delete) {
      deleteHalfedge(he);
    }
    if (!to_delete.empty()) {
      std::cout << "Cleaned up " << to_delete.size() << " dangling halfedges" << std::endl;
    }
  }

  // 全faceのhalfedgeリストを再構築（nullptr参照を除外）
  void rebuildAllFaceHalfedgeLists() {
    for (auto& fc : faces_) {
      std::list<std::shared_ptr<HalfedgeL>> valid_halfedges;
      for (auto& he : fc->halfedges()) {
        if (he && he->vertex() && he->face()) {
          valid_halfedges.push_back(he);
        }
      }
      fc->deleteHalfedges();
      for (auto& he : valid_halfedges) {
        fc->addHalfedge(he);
      }
    }
    std::cout << "Rebuilt all face halfedge lists" << std::endl;
  }

  bool isConnectivity() const { return isConnectivity_; };
  void setConnectivity(bool f) { isConnectivity_ = f; };

  //
  // 折れ線を考慮していないスムースシェーディング用
  // (ハーフエッジを使わない例)
  //
  void calcSmoothVertexNormal() {

    if (normals_.size()) return;

    // 面の数 保存用
    std::vector<int> n_vf;
    n_vf.resize(vertices_size());

    // 面積計算用
    std::vector<double> area(vertices_size());
    // normal 計算用
    std::vector<Eigen::Vector3d> nmvec(vertices_size());

    // 初期化
    // id の付け直し
    for (int i = 0; i < vertices_size(); ++i) {
      n_vf[i] = 0;
      nmvec[i] = Eigen::Vector3d::Zero();
    }

    for (auto& fc : faces_) {
      double a = fc->area();
      for (auto& he : fc->halfedges()) {
        n_vf[he->vertex()->id()]++;
        area[he->vertex()->id()] += a;
        Eigen::Vector3d nrm = fc->normal();
        nrm *= a;
        nmvec[he->vertex()->id()] += nrm;
      }
    }

    std::vector<std::shared_ptr<NormalL>> nm_array;
    nm_array.resize(vertices_size());
    for (int i = 0; i < vertices_size(); ++i) {
      double f = (double)n_vf[i];
      nmvec[i] /= (f * area[i]);
      nmvec[i].normalize();
      auto nm = addNormal(nmvec[i]);
      nm_array[i] = nm;
    }

    for (auto& fc : faces_) {
      for (auto& he : fc->halfedges()) {
        he->setNormal(nm_array[he->vertex()->id()]);
      }
    }
  };

  void computeBB( Eigen::Vector3d& bbmin, Eigen::Vector3d& bbmax ) {
    int i = 0;
    for (auto& vt : vertices_) {
      Eigen::Vector3d& p = vt->point();
      if (i) {
        if (p.x() > bbmax.x()) bbmax.x() = p.x();
        if (p.x() < bbmin.x()) bbmin.x() = p.x();
        if (p.y() > bbmax.y()) bbmax.y() = p.y();
        if (p.y() < bbmin.y()) bbmin.y() = p.y();
        if (p.z() > bbmax.z()) bbmax.z() = p.z();
        if (p.z() < bbmin.z()) bbmin.z() = p.z();
      } else {
        bbmax = p;
        bbmin = p;
      }
      ++i;
    }
  };

  void normalize(Eigen::Vector3d& center, double maxlen) {
    for (auto& vt : vertices_) {
      Eigen::Vector3d p1 = vt->point() - center;
      p1 /= maxlen;
      vt->setPoint(p1);
    }
  };

  void normalize() {
    if (isNormalized()) return;

    std::cout << "normalize ... ";
    Eigen::Vector3d vmax, vmin;
    int i = 0;
    for (auto& vt : vertices_) {
      Eigen::Vector3d& p = vt->point();
      if (i) {
        if (p.x() > vmax.x()) vmax.x() = p.x();
        if (p.x() < vmin.x()) vmin.x() = p.x();
        if (p.y() > vmax.y()) vmax.y() = p.y();
        if (p.y() < vmin.y()) vmin.y() = p.y();
        if (p.z() > vmax.z()) vmax.z() = p.z();
        if (p.z() < vmin.z()) vmin.z() = p.z();
      } else {
        vmax = p;
        vmin = p;
      }
      ++i;
    }

    center_ = (vmax + vmin) * .5;

    Eigen::Vector3d len(vmax - vmin);
    double maxl = (std::fabs(len.x()) > std::fabs(len.y())) ? std::fabs(len.x())
      : std::fabs(len.y());
    maxl = (maxl > std::fabs(len.z())) ? maxl : std::fabs(len.z());
    setMaxLength(maxl);

    normalize(center_, maxl);

    setIsNormalized(true);

    std::cout << "done." << std::endl;
  };

  // void unnormalize();
  void unnormalize() {
    if (!isNormalized()) return;

    std::cout << "unnormalize ... ";

    for (auto& vt : vertices_) {
      Eigen::Vector3d p(vt->point());
      p *= maxLength();
      p += center_;
      vt->setPoint(p);
    }

    setIsNormalized(false);

    std::cout << "done." << std::endl;
  };

  // void normalizeTexcoord();
  void normalizeTexcoord() {
    Eigen::Vector2d vmax, vmin;
    int i = 0;
    for (auto& tc : texcoords_) {
      Eigen::Vector3d& p0 = tc->point();
      Eigen::Vector2d p(p0.x(), p0.y());
      if (i) {
        if (p.x() > vmax.x()) vmax.x() = p.x();
        if (p.x() < vmin.x()) vmin.x() = p.x();
        if (p.y() > vmax.y()) vmax.y() = p.y();
        if (p.y() < vmin.y()) vmin.y() = p.y();
      } else {
        vmax = p;
        vmin = p;
      }
      ++i;
    }

    double xlen = vmax.x() - vmin.x();
    double ylen = vmax.y() - vmin.y();

    for (auto& tc : texcoords_) {
      Eigen::Vector3d& p = tc->point();
      Eigen::Vector3d q((p.x() - vmin.x()) / xlen, (p.y() - vmin.y()) / ylen, .0);
      tc->setPoint(q);
    }
  };

  void calcAllFaceNormals() {
    for (auto& fc : faces_)
      fc->calcNormal();
  };

  void createBLoop() {
    // needs connectivity
    createConnectivity(true);
    std::shared_ptr<VertexL> vt = findBoundaryVertex();
    if (vt == nullptr) return;
    createBLoop(vt);
  };

  //void createBLoop(VertexL*);
  //
  // create boundary loop
  // sv: starting vertex
  void createBLoop(std::shared_ptr<VertexL> sv) {
    // needs connectivity
    createConnectivity(true);

    if (isBoundary(sv) == false) return;

    if (!(emptyBLoop())) deleteAllBLoops();

    std::shared_ptr<BLoopL> bl = addBLoop();

    std::shared_ptr<VertexL> vt = sv;    // current vertex
    std::shared_ptr<VertexL> pv = nullptr;  // previous vertex
    do {
      std::cout << *vt << std::endl;

      bl->addVertex(vt);
      bl->addIsCorner(false);  // default

      // next boundary vertex
      VertexLCirculator vc(vt);
      std::shared_ptr<VertexL> vv = vc.beginVertexL();
      do {
        vv = vc.nextVertexL();
        if ((isBoundary(vv)) && (vv != pv)) break;
      } while ((vv != vc.firstVertexL()) && (vv != nullptr));

      pv = vt;
      vt = vv;

    } while (vt != sv);

    // set first vertex to corner.
    bl->setCorner(0, true);

    // optimize boundary corner
    bl->optimize(4);

    std::cout << "BLoop created." << std::endl;
  }

  bool isVerticesSelected() {
    for (auto& vt : vertices_)
      if (vt->isSelected()) return true;
    return false;
  };

  bool isFacesSelected() {
    for (auto& fc : faces_)
      if (fc->isSelected()) return true;
    return false;
  };

  // select
  void clearAllVerticesSelected() {
    for (auto& vt : vertices_) vt->setSelected(false);
  };

  void setAllVerticesSelected() {
    for (auto& vt : vertices_) vt->setSelected(true);
  };

  void clearAllFacesSelected() {
    for (auto& fc : faces_) fc->setSelected(false);
  };

  void setAllFacesSelected() {
    for (auto& fc : faces_) fc->setSelected(true);
  };

  //
  // create a new mesh with reordering vertices according to an array "order"
  //
  void reorderVertices(std::vector<int>& order) {
    int n_vt = vertices_.size();

    // store old vertices
    std::vector<std::shared_ptr<VertexL> > old_vt;
    int id = 0;
    for (auto& vt : vertices_) {
      old_vt.push_back(vt);
      vt->setID(id++);
    }

    // create vertices according to the order array and add vertex lists
    std::vector<std::shared_ptr<VertexL> > new_vt;
    id = 0;
    std::vector<int> new_id(n_vt);
    for (int i = 0; i < n_vt; ++i) {
      std::shared_ptr<VertexL> nvt = addVertex(old_vt[order[i]]->point());
      new_id[order[i]] = id;
      nvt->setID(id++);
      new_vt.push_back(nvt);
    }

    // reconnect the halfedge's vertices
    for (auto& fc : faces_) {
      for (auto& he : fc->halfedges()) {
        he->setVertex(new_vt[new_id[he->vertex()->id()]]);
      }
    }

    // delete former vertices
    auto vt_iter = vertices_.begin();
    for (int i = 0; i < n_vt; ++i) {
      auto nvt = vt_iter;
      ++vt_iter;
      deleteVertex(*nvt);
    }
  };

  //
  // reordering new faces according to an array "indices"
  //
  void reorderIndices(unsigned int n_indices, unsigned int* indices) {
    int n_fc = faces_.size();

    // store vertex pointers
    std::vector<std::shared_ptr<VertexL>> old_vt;
    int id = 0;
    for (auto& vt : vertices_) {
      old_vt.push_back(vt);
      vt->setID(id++);
    }

    // create new faces
    id = 0;
    unsigned int i = 0;
    while (i < n_indices) {
      std::shared_ptr<FaceL> fc = addFace();
      fc->setID(id++);

      std::shared_ptr<VertexL> vt;
      // HalfedgeL* he;
      vt = old_vt[indices[i]];
      ++i;
      (void)addHalfedge(fc, vt, nullptr, nullptr);
      vt = old_vt[indices[i]];
      ++i;
      (void)addHalfedge(fc, vt, nullptr, nullptr);
      vt = old_vt[indices[i]];
      ++i;
      (void)addHalfedge(fc, vt, nullptr, nullptr);
    }

    // delete former faces
    auto fc_iter = faces_.begin();
    for (int i = 0; i < n_fc; ++i) {
      auto nfc = fc_iter;
      ++fc_iter;
      deleteFace(*nfc);
    }
  };

  unsigned int texID() const { return texID_; };
  //
  // set Texture id to all selected faces
  //
  // selected:
  //   - true: set only selected faces
  //   - false: set all faces
  //
  void setTexIDToFaces(int id, bool selected) {
    for (auto& fc : faces_) {
      if (selected) {
        if (fc->isSelected()) fc->setTexID(id);
      } else
        fc->setTexID(id);
    }
  };

  void changeTexID(int id0, int id1) {
    texID_ = id1;
    for (auto& fc : faces_) {
      if (fc->texID() == id0) fc->setTexID(id1);
    }
  };

  void copyTexcoordToVertex(std::vector<Eigen::Vector3d>& p) {
    auto tc = texcoords_.begin();
    int i = 0;
    for (auto& vt : vertices_) {
      p[i] = vt->point();
      vt->setPoint((Eigen::Vector3d&)(*tc)->point());
      tc++;
      ++i;
    }
  };

  void copyVertex(std::vector<Eigen::Vector3d>& p) {
    int i = 0;
    for (auto& vt : vertices_) {
      vt->setPoint(p[i]);
      ++i;
    }
  };

  void resetVertexID() {
    int i = 0;
    for (auto& vt : vertices_) {
      vt->setID(i);
      ++i;
    }
  };

  void resetHalfedgeID() {
    int i = 0;
    for (auto& fc : faces_) {
      for (auto& he : fc->halfedges()) {
        he->setID(i);
        ++i;
      }
    }
    assert(halfedges_size() == i);
  };

  void resetFaceID() {
    int i = 0;
    for (auto& fc : faces_) {
      fc->setID(i);
      ++i;
    }
  };

  void print() {
    for (auto& fc : faces_) fc->print();
  };

  void printInfo() {
    std::cout << "mesh "
              << " ";
    if (vertices_.size())
      std::cout << " v " << (unsigned int)vertices().size() << " ";
    if (normals_.size())
      std::cout << " n " << (unsigned int)normals().size() << " ";
    if (texcoords_.size())
      std::cout << " t " << (unsigned int)texcoords().size() << " ";
    if (faces_.size()) std::cout << " f " << (unsigned int)faces_.size() << " ";
    if (bloops_.size())
      std::cout << " bl " << (unsigned int)bloops_.size() << " ";
    std::cout << std::endl;
  };

private:

  // vertices

  std::shared_ptr<VertexL> addVertex() {
    int32_t id = v_id_++;
    std::shared_ptr<VertexL> vt = std::make_shared<VertexL>(id);
    vt->setIter(vertices_.insert(vertices_.end(), vt));
    return vt;
  };
  
  int v_id_;
  std::list<std::shared_ptr<VertexL> > vertices_;

  // normals (for smooth shading)

  std::shared_ptr<NormalL> addNormal() {
    int32_t id = n_id_++;
    std::shared_ptr<NormalL> nm = std::make_shared<NormalL>(id);
    nm->setIter(normals_.insert(normals_.end(), nm));
    return nm;
  };

  int n_id_;
  std::list<std::shared_ptr<NormalL> > normals_;

  // texcoords

  std::shared_ptr<TexcoordL> addTexcoord() {
    int32_t id = t_id_++;
    std::shared_ptr<TexcoordL> tc = std::make_shared<TexcoordL>(id);
    tc->setIter(texcoords_.insert(texcoords_.end(), tc));
    return tc;
  };

  int t_id_;
  std::list<std::shared_ptr<TexcoordL> > texcoords_;

  // halfedges

  std::shared_ptr<HalfedgeL> addHalfedge() {
    int32_t id = h_id_++;
    std::shared_ptr<HalfedgeL> he = std::make_shared<HalfedgeL>(id);
    he->setMeshIter(halfedges_.insert(halfedges_.end(), he));
    // he->setMeshEnd(halfedges_.end());
    return he;
  };

  int h_id_;
  std::list<std::shared_ptr<HalfedgeL> > halfedges_;

  // faces

  int f_id_;
  std::list<std::shared_ptr<FaceL> > faces_;

  // edges (define if needed)

  std::shared_ptr<EdgeL> addEdge() {
    int32_t id = e_id_++;
    std::shared_ptr<EdgeL> ed = std::make_shared<EdgeL>(id);
    ed->setIter(edges_.insert(edges_.end(), ed));
    return ed;
  };

  int e_id_;
  std::list<std::shared_ptr<EdgeL> > edges_;

  // loops (define if needed)
  int l_id_;
  std::list<std::shared_ptr<LoopL> > loops_;

  // boundary loops (define if needed)
  int bl_id_;
  std::list<std::shared_ptr<BLoopL> > bloops_;

  bool isConnectivity_;

  bool isNormalized_;
  Eigen::Vector3d center_;
  double max_length_;

  // texture
  unsigned int texID_;

};

#endif  // _MESHL_HXX
