////////////////////////////////////////////////////////////////////
//
// $Id: HalfedgeL.hxx 2025/07/17 23:53:17 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _HALFEDGEL_HXX
#define _HALFEDGEL_HXX 1

#include "envDep.h"
#include "mydef.h"

#include "myEigen.hxx"

#include <list>
#include <memory>
//using namespace std;

#include "VMProc.hxx"

#include "NodeL.hxx"
#include "VertexL.hxx"
#include "NormalL.hxx"
#include "TexcoordL.hxx"
class EdgeL;
class FaceL;

class HalfedgeL : public NodeL, public std::enable_shared_from_this<HalfedgeL> {

public:

  HalfedgeL() : NodeL() {init();};
  HalfedgeL( int id ) : NodeL(id) { init(); };
  ~HalfedgeL(){};

  void init() {
    vertex_ = nullptr;
    normal_ = nullptr;
    texcoord_ = nullptr;
    mate_ = nullptr;
    edge_ = nullptr;
    face_ = nullptr;
    f_hlist_ = nullptr;
    f_iter_ = std::list<std::shared_ptr<HalfedgeL> >::iterator();
    m_iter_ = std::list<std::shared_ptr<HalfedgeL> >::iterator();
  }

  std::list<std::shared_ptr<HalfedgeL> >::iterator f_begin() const { return f_hlist_->begin(); }
  std::list<std::shared_ptr<HalfedgeL> >::iterator f_end() const { return f_hlist_->end(); }

  std::shared_ptr<HalfedgeL> next() const {
    auto h_iter = f_iter_;
    h_iter++;
    return (h_iter != f_end()) ? *h_iter : *(f_begin());
  };

  std::shared_ptr<HalfedgeL> prev() const {
    auto h_iter = f_iter_;
    if ( h_iter != f_begin() ) --h_iter;
    else {
      h_iter = f_end();
      --h_iter;
    }
    return *(h_iter);
  };

  // this の前に挿入
  std::list<std::shared_ptr<HalfedgeL> >::iterator binsert( std::shared_ptr<HalfedgeL> new_he ) {
    auto h_iter = f_iter();
    return f_hlist_->insert(h_iter, new_he);
  };

  // this の後に挿入
  std::list<std::shared_ptr<HalfedgeL> >::iterator ainsert( std::shared_ptr<HalfedgeL> new_he ) {
    auto h_iter = f_iter();
    if ( h_iter == f_end() ) h_iter = f_begin();
    else h_iter++;
    return f_hlist_->insert(h_iter, new_he);
  };
  
  
  // iterator of MeshL's halfedge list
  std::list<std::shared_ptr<HalfedgeL> >::iterator meshIter() const { return m_iter_; }
  void setMeshIter(std::list<std::shared_ptr<HalfedgeL> >::iterator iter) { m_iter_ = iter; }

// vertex
  std::shared_ptr<VertexL> vertex() const { return vertex_; }
  void setVertex( std::shared_ptr<VertexL> vt ) { vertex_ = vt; }

  std::shared_ptr<VertexL> next_vertex() const { return next()->vertex(); }
  std::shared_ptr<VertexL> prev_vertex() const { return prev()->vertex(); }
  std::shared_ptr<VertexL> opposite_vertex() const { return prev_vertex(); }

  // mate
  std::shared_ptr<HalfedgeL> mate() const { return mate_; }
  bool mate_valid(std::shared_ptr<HalfedgeL> he) const {
    return ((vertex() == he->next_vertex()) && (next_vertex() == he->vertex()));
  }
  void setMate(std::shared_ptr<HalfedgeL> he) { mate_ = he; }
  void setBothMate(std::shared_ptr<HalfedgeL> he) { setMate(he); he->setMate(shared_from_this()); }
  //boundary
  bool isBoundary() const { return (mate_ == nullptr) ? true : false; }

  // normal
  bool isNormal() const { return ( normal_ != nullptr ) ? true : false; }
  std::shared_ptr<NormalL> normal() const { return normal_; }
  void setNormal( std::shared_ptr<NormalL> nm ) { normal_ = nm; }

  // texcoord
  bool isTexcoord() const { return ( texcoord_ != nullptr ) ? true : false; }
  std::shared_ptr<TexcoordL> texcoord() const { return texcoord_; }
  void setTexcoord( std::shared_ptr<TexcoordL> tc ) { texcoord_ = tc; }

  // face
  std::shared_ptr<FaceL> face() const { return face_; }
  void setFace( std::shared_ptr<FaceL> fc ) {
    face_ = fc;
    if (f_hlist_ != nullptr) {
      f_iter_ = f_end();
    }
  };

  void setFaceandFIter(std::shared_ptr<FaceL> fc,
                       std::list<std::shared_ptr<HalfedgeL>>::iterator iter,
                       std::list<std::shared_ptr<HalfedgeL>>& f_hlist) {
    setFHalfedges(f_hlist);         // 最初に f_hlist_ を設定
    setFace(fc);                    // その後で setFace を呼び出す
    setFIter(iter);
  };

  // iterator of FaceL
  std::list<std::shared_ptr<HalfedgeL> >::iterator f_iter() const { return f_iter_; }
  void setFIter(std::list<std::shared_ptr<HalfedgeL> >::iterator iter) {
    //if (f_iter_ != f_end()) return;
    f_iter_ = iter;
  };

  void setFHalfedges(std::list<std::shared_ptr<HalfedgeL> >& f_hlist) { 
    f_hlist_ = &(f_hlist);
  };

  // edge
  void setEdge( std::shared_ptr<EdgeL> edge ) { edge_ = edge; }
  std::shared_ptr<EdgeL> edge() const { return edge_; }

  // length
  double length() const {
    return Eigen::Vector3d(vertex()->point() - next()->vertex()->point()).norm();
  }

  // length
  double param_length() const {
    return Eigen::Vector3d(texcoord()->point() - next()->texcoord()->point()).norm();
  }

  // for texcoord
  std::shared_ptr<HalfedgeL> findNextHalfedge(Eigen::Vector2d& v0, Eigen::Vector2d& v1) {
    auto he = this->next();
    auto start_he = shared_from_this();
    do {
      Eigen::Vector3d& sv0 = he->texcoord()->point();
      Eigen::Vector3d& ev0 = he->next()->texcoord()->point();
      Eigen::Vector2d sv( sv0.x(), sv0.y() );
      Eigen::Vector2d ev( ev0.x(), ev0.y() );
      if ( isLineSegmentCrossing2d( sv, ev, v0, v1 ) == true ) {
        return he->mate();
      }
      he = he->next();
    } while ( he != start_he );

    return nullptr;
  }

  // re-attaching vertex's halfedge to the end.
  std::shared_ptr<HalfedgeL> reset() {
    auto he = shared_from_this();
    if ( he->mate() == nullptr ) return he;
    auto start_he = he;
    do {
      he = he->mate()->next();
    } while ( (he->mate() != nullptr) && ( he != start_he) );

    return he;
  }


private:

  // vertex
  std::shared_ptr<VertexL> vertex_;

  // normal
  std::shared_ptr<NormalL> normal_;

  // texture coordinate
  std::shared_ptr<TexcoordL> texcoord_;

  std::shared_ptr<HalfedgeL> mate_;
  std::shared_ptr<FaceL> face_; // back face

  std::shared_ptr<EdgeL> edge_;

  // list iterator of FaceL
  std::list<std::shared_ptr<HalfedgeL> >::iterator f_iter_;
  std::list<std::shared_ptr<HalfedgeL> >* f_hlist_;

  // iterator of MeshL's halfedge list
  std::list<std::shared_ptr<HalfedgeL> >::iterator m_iter_;

};

#endif // _HALFEDGEL_HXX
