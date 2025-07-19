////////////////////////////////////////////////////////////////////
//
// $Id: EdgeL.hxx 2025/07/17 21:57:33 kanai Exp 
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _EDGEL_HXX
#define _EDGEL_HXX 1

#include "envDep.h"
#include "mydef.h"

#include <list>
#include <memory>
//using namespace std;

#include "NodeL.hxx"

class VertexL;
#include "HalfedgeL.hxx"

class MeshL;

// Edge と頂点，面との関係
//            ev
//            |
//            |
//    lf  <-  |  -> rf
//            |
//            |
//            sv
class EdgeL : public NodeL, public std::enable_shared_from_this<EdgeL> {

public:

  EdgeL() : NodeL() { init(); };
  EdgeL( int id ) : NodeL(id) { init(); };
//    EdgeL( VertexL* sv, VertexL* ev, HalfedgeL* lhe, HalfedgeL* rhe ) {
//      setVertices( sv, ev );
//      setLHalfedge( lhe ); setRHalfedge( lhe );
//    };
  ~EdgeL(){};
  void init() {
//     iter_ = nullptr;
    sv_ = nullptr; ev_ = nullptr;
    lhe_ = nullptr; rhe_ = nullptr;
    iter_ = std::list<std::shared_ptr<EdgeL> >::iterator();
  };

  // vertex
  std::shared_ptr<VertexL> sv() const { return sv_; }
  std::shared_ptr<VertexL> ev() const { return ev_; }
  void setSVertex( std::shared_ptr<VertexL> sv ) { sv_ = sv; }
  void setEVertex( std::shared_ptr<VertexL> ev ) { ev_ = ev; }
  void setVertices( std::shared_ptr<VertexL> sv, std::shared_ptr<VertexL> ev ) {
    setSVertex( sv ); setEVertex( ev );
  };

  // halfedge
  std::shared_ptr<HalfedgeL> lhe() const { return lhe_; }
  std::shared_ptr<HalfedgeL> rhe() const { return rhe_; }
  void setLHalfedge( std::shared_ptr<HalfedgeL> lhe ) {
    lhe_ = lhe;
    if ( lhe_ != nullptr ) {
      lhe_->setEdge( shared_from_this() );
    }
  }
  void setRHalfedge( std::shared_ptr<HalfedgeL> rhe ) {
    rhe_ = rhe;
    if ( rhe_ != nullptr ) {
      rhe_->setEdge( shared_from_this() );
    }
  }

  bool rhe_valid( std::shared_ptr<HalfedgeL> rhe ) {
    // reverse order
    if ( (sv_ == rhe->next()->vertex()) && (ev_ == rhe->vertex()) ) return true; 
    return false;
  };
  
  bool isBoundary() const { return ( (lhe_) && (rhe_) ) ? false : true; };
  
  // iter
  void setIter(std::list<std::shared_ptr<EdgeL> >::iterator iter) { iter_ = iter; }
  std::list<std::shared_ptr<EdgeL> >::iterator iter() const { return iter_; }

  // void setMesh( MeshL* );

private:

  std::shared_ptr<VertexL> sv_;
  std::shared_ptr<VertexL> ev_;
  std::shared_ptr<HalfedgeL> lhe_;
  std::shared_ptr<HalfedgeL> rhe_;

  // list iterator of MeshL
  std::list<std::shared_ptr<EdgeL> >::iterator iter_;

};

class EdgeList {

  std::list<std::shared_ptr<EdgeL>> l_;

public:

  EdgeList() {};
  ~EdgeList() {};

  std::shared_ptr<EdgeL> findEdge( std::shared_ptr<VertexL> sv, std::shared_ptr<VertexL> ev ) {
    // foreach ( std::list<EdgeL*>, l_, ed )
    for (auto& ed : l_) {
      if ( (ed->sv() == sv) && (ed->ev() == ev) )
        return ed;
      if ( (ed->ev() == sv) && (ed->sv() == ev) )
        return ed;
    }
    return nullptr;
  }

  void push_back( std::shared_ptr<EdgeL> ed ) { l_.push_back(ed); }

//private:

//  std::shared_ptr<VertexL> sv_;
//  std::shared_ptr<VertexL> ev_;
//  std::shared_ptr<HalfedgeL> lhe_;
//  std::shared_ptr<HalfedgeL> rhe_;

  // list iterator of MeshL
//  std::list<EdgeL*>::iterator iter_;

};

#endif // _EDGEL_HXX
