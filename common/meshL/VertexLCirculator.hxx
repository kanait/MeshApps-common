////////////////////////////////////////////////////////////////////
//
// $Id: VertexLCirculator.hxx 2025/07/06 19:21:41 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _VERTEXLCIRCULATOR_HXX
#define _VERTEXLCIRCULATOR_HXX

#include "VertexL.hxx"
#include "HalfedgeL.hxx"
#include "FaceL.hxx"
#include <memory>

class VertexLCirculator {

  std::shared_ptr<VertexL> vt_;

protected:

  std::shared_ptr<HalfedgeL> temp_halfedge_;

public:

  VertexLCirculator() { clear(); };
  VertexLCirculator(std::shared_ptr<VertexL> vt) { clear(); setVertex(vt); }
  ~VertexLCirculator() {};

  void clear() { vt_ = nullptr; temp_halfedge_ = nullptr; };
  void setVertex(std::shared_ptr<VertexL> vt) { vt_ = vt; }

  //
  // vertex -> face
  //
  std::shared_ptr<FaceL> beginFaceL() {
    temp_halfedge_ = vt_->halfedge();
    assert( temp_halfedge_ );
    return temp_halfedge_->face();
  }

  std::shared_ptr<FaceL> nextFaceL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->prev()->mate();
    if ( temp_halfedge_ == nullptr ) return nullptr;
    //cout << "\t\t next face " << temp_halfedge_->face()->id() << endl;
    return temp_halfedge_->face();
  }

  std::shared_ptr<FaceL> firstFaceL() {
    return vt_->halfedge()->face();
  }

  std::shared_ptr<FaceL> lastFaceL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate()->face();
  }

  //
  // vertex -> vertex
  //
  std::shared_ptr<VertexL> beginVertexL() {
    temp_halfedge_ = vt_->halfedge();
    assert( temp_halfedge_ );
    return temp_halfedge_->next()->vertex();
  }

  std::shared_ptr<VertexL> nextVertexL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    if ( he == nullptr ) return nullptr;
    temp_halfedge_ = he->prev()->mate();
    if ( temp_halfedge_ == nullptr )
      {
        // last vertex
        return he->prev()->vertex();
      }
    //return nullptr;
    //cout << "\t\t next face " << temp_halfedge_->face()->id() << endl;
    return temp_halfedge_->next()->vertex();
  }

  std::shared_ptr<VertexL> firstVertexL() {
    return vt_->halfedge()->next()->vertex();
  }

  std::shared_ptr<VertexL> lastVertexL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate()->next()->vertex();
  }

  //
  // vertex -> halfedge
  //
  std::shared_ptr<HalfedgeL> beginHalfedgeL() {
    temp_halfedge_ = vt_->halfedge();
    assert( temp_halfedge_ );
    return temp_halfedge_;
  }

  std::shared_ptr<HalfedgeL> nextHalfedgeL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->prev()->mate();
    if ( temp_halfedge_ == nullptr ) return nullptr;
    return temp_halfedge_;
  }

  std::shared_ptr<HalfedgeL> prevHalfedgeL() {
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->mate()->next();
    if ( temp_halfedge_ == nullptr ) return nullptr;
    return temp_halfedge_;
  }

  std::shared_ptr<HalfedgeL> firstHalfedgeL() {
    return vt_->halfedge();
  }

  std::shared_ptr<HalfedgeL> lastHalfedgeL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate()->next();
  }

  //
  //  vertex -> reverse halfedge
  //
  std::shared_ptr<HalfedgeL> beginRevHalfedgeL() {
    assert( vt_->halfedge() );
    temp_halfedge_ = vt_->halfedge()->prev();
    return temp_halfedge_;
  }

  std::shared_ptr<HalfedgeL> nextRevHalfedgeL() {
    if ( temp_halfedge_->mate() == nullptr ) return nullptr;
    std::shared_ptr<HalfedgeL> he = temp_halfedge_;
    temp_halfedge_ = he->mate()->prev();
    return temp_halfedge_;
  }

  std::shared_ptr<HalfedgeL> firstRevHalfedgeL() {
    return vt_->halfedge()->prev();
  }

  std::shared_ptr<HalfedgeL> lastRevHalfedgeL() {
    if ( vt_->halfedge()->mate() == nullptr ) return nullptr;
    return vt_->halfedge()->mate();
  }

  int num_vertices() {
    int count=0;
    std::shared_ptr<VertexL> vt = beginVertexL();
    if (vt == nullptr) return 0;
    std::shared_ptr<VertexL> first_vt = firstVertexL();
//      cout << "(vv) vt " << vt_->id() << " ";
    do { 
      count++; 
      std::cout << vt->id() << " "; 
      vt = nextVertexL();
    } while ( (vt != first_vt) && (vt != nullptr) );
//      cout << endl;
    return count;
  }

  int num_faces() {
    int count=0;
    std::shared_ptr<FaceL> fc = beginFaceL();
    if (fc == nullptr) return 0;
    std::shared_ptr<FaceL> first_fc = firstFaceL();
//      cout << "(vf) vt " << vt_->id() << " ";
    do { 
      count++; 
      std::cout << fc->id() << " "; 
      fc = nextFaceL(); 
    } while ( (fc != first_fc) && (fc != nullptr) );
//      cout << endl;
    return count;
  }

  void setfirstHalfedge(std::shared_ptr<HalfedgeL> he) {
    vt_->setHalfedge(he);
  }

};


#endif // _VERTEXLCIRCULATOR_HXX
