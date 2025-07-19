////////////////////////////////////////////////////////////////////
//
// $Id: LoopL.hxx 2022/05/22 21:11:27 kanai Exp $
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _LOOPL_HXX
#define _LOOPL_HXX 1

#include "envDep.h"

#include <vector>
#include <list>
#include <memory>
//using namespace std;

class VertexL;
class HalfedgeL;

#include "NodeL.hxx"

class LoopL : public NodeL {

public:

  LoopL() : NodeL() { init(); };
  LoopL( int id ) : NodeL(id) { init(); };
  virtual ~LoopL() {
    vertices_.clear();
    halfedges_.clear();
  };

  void init() {

    vertices_.clear();
    halfedges_.clear();
    cur_v_id_ = 0; cur_he_id_ = 0;
    isClosedLoop_ = false;
  };

  void reset() { resetVertex(); resetHalfedge(); };

  //
  // vertices
  //
  void addVertex( std::shared_ptr<VertexL> vt ) {
    vertices_.push_back( vt );
  }
  void clearVertices() { vertices_.clear(); cur_v_id_ = 0; };

  std::shared_ptr<VertexL> vertex( int i ) {
    if ( (i < 0) || (i >= (int) vertices_.size() ) ) return nullptr;
    return vertices_[i];
  };
  std::shared_ptr<VertexL> beginVertex() {
    cur_v_id_ = 0;
    return vertices_[cur_v_id_];
  }
  std::shared_ptr<VertexL> firstVertex() { return vertices_[0]; }

  std::shared_ptr<VertexL> nextVertex() {
    if ( cur_v_id_ != ((int)vertices_.size() - 1) ) ++cur_v_id_;
    else { // cur_v_id_ == (vertices_.size() - 1)
      if ( isClosedLoop() == false ) return nullptr;
      cur_v_id_ = 0;
    }
    return vertices_[cur_v_id_];
  }
  std::shared_ptr<VertexL> prevVertex() {
    if ( cur_v_id_ != 0 ) --cur_v_id_;
    else {
      if ( isClosedLoop() == false ) return nullptr;
      cur_v_id_ = (int)vertices_.size()-1;
    }
    return vertices_[cur_v_id_];
  }
  void resetVertex() { cur_v_id_ = 0; };

  //
  // halfedges
  //
  void addHalfedge( std::shared_ptr<HalfedgeL> he ) {
    halfedges_.push_back( he );
  }
  void clearHalfedges() { halfedges_.clear(); cur_he_id_ = 0; }
    
  std::shared_ptr<HalfedgeL> halfedge( int i ) {
    if ( (i < 0) || (i >= (int)halfedges_.size() ) ) return nullptr;
    return halfedges_[i];
  }
  std::shared_ptr<HalfedgeL> beginHalfedge() {
    cur_he_id_ = 0;
    return halfedges_[cur_he_id_];
  }
  std::shared_ptr<HalfedgeL> firstHalfedge() {
    return (halfedges_.size()) ? halfedges_[0] : nullptr;
  }

  std::shared_ptr<HalfedgeL> nextHalfedge() {
    if ( cur_he_id_ != ((int)halfedges_.size() - 1) ) ++cur_he_id_;
    else {
      if ( isClosedLoop() == false ) return nullptr;
      cur_he_id_ = 0;
    }
    return halfedges_[cur_he_id_];
  }
  std::shared_ptr<HalfedgeL> prevHalfedge() {
    if ( cur_he_id_ != 0 ) --cur_he_id_;
    else {
      if ( isClosedLoop() == false ) return nullptr;
      cur_he_id_ = (int)halfedges_.size()-1;
    }
    return halfedges_[cur_he_id_];
  }
  void resetHalfedge() { cur_he_id_ = 0; };

  // iter
  void setIter(std::list<std::shared_ptr<LoopL> >::iterator iter) { iter_ = iter; }
  std::list<std::shared_ptr<LoopL> >::iterator iter() const { return iter_; }

  //
  // loop flag
  //
  bool isClosedLoop() const { return isClosedLoop_; };
  void setIsClosedLoop( bool f ) { isClosedLoop_ = f; };

  std::vector<std::shared_ptr<VertexL> >& vertices() { return vertices_; }
  std::vector<std::shared_ptr<HalfedgeL> >& halfedges() { return halfedges_; }

private:

  // loop vertices
  int cur_v_id_;
  std::vector<std::shared_ptr<VertexL> > vertices_;

  // loop half edges (define if needed)
  int cur_he_id_;
  std::vector<std::shared_ptr<HalfedgeL> > halfedges_;

  bool isClosedLoop_;

  // list iterator of MeshL
  std::list<std::shared_ptr<LoopL> >::iterator iter_;

};

#endif // _LOOPL_HXX
