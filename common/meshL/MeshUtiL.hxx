////////////////////////////////////////////////////////////////////
//
// $Id: MeshUtiL.hxx 2025/07/06 19:22:59 kanai Exp 
//
// Copyright (c) 2022 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _MESHUTIL_HXX
#define _MESHUTIL_HXX 1

#include "envDep.h"
#include <memory>

#include "VertexL.hxx"
#include "VertexLCirculator.hxx"

//
// VertexL utility
//

inline std::shared_ptr<HalfedgeL> findHalfedge( std::shared_ptr<VertexL> o, std::shared_ptr<VertexL> vt ) {
  VertexLCirculator vc( o );
  std::shared_ptr<HalfedgeL> he = vc.beginHalfedgeL();
  do {
    if ( he->next()->vertex() == vt ) return he;

    std::shared_ptr<HalfedgeL> mate = he->mate();
    if ( mate )
      {
        if ( mate->vertex() == vt ) return mate;
      }
    he = vc.nextHalfedgeL();
  } while ( (he != nullptr) && (he != vc.firstHalfedgeL()) );

  return nullptr;
}

inline void reset_halfedge( std::shared_ptr<VertexL> vt ) {
  if ( vt->halfedge() == nullptr ) return;
  std::shared_ptr<HalfedgeL> he = vt->halfedge();
  if ( he->mate() == nullptr ) return;
  do {
    he = he->mate()->next();
  } while ( (he->mate() != nullptr) && ( he != vt->halfedge()) );
  vt->setHalfedge(he);
}

inline bool isBoundary( std::shared_ptr<VertexL> vt ) {
  VertexLCirculator vc( vt );
  std::shared_ptr<HalfedgeL> he = vc.beginHalfedgeL();
  do {
    he = vc.nextHalfedgeL();
    if ( he == nullptr ) return true;
  } while ( he != vc.firstHalfedgeL() );

  return false;
}

inline int valence( std::shared_ptr<VertexL> ovt ) {
  int count = 0;
  VertexLCirculator vc( ovt );
  std::shared_ptr<VertexL> vt = vc.beginVertexL();
  do {
    count++;
    vt = vc.nextVertexL();
  } while ( (vt != vc.firstVertexL()) && (vt != nullptr) );

  return count;
}

inline void calcVertexNormal( std::shared_ptr<VertexL> vt, Eigen::Vector3d& nv ) {
  nv = Eigen::Vector3d::Zero();

  int i = 0;
  VertexLCirculator vc( vt );
  std::shared_ptr<FaceL> fc = vc.beginFaceL();
  if ( fc == nullptr ) return;
  do {
    // add face normal
    ++i;
    nv += fc->normal();
    fc = vc.nextFaceL();
  } while ( (fc != vc.firstFaceL()) && (fc != nullptr) );

  // divided by its valence
  nv /= (double)i;
  nv.normalize();
}

inline void printNeighborFaces(std::shared_ptr<VertexL> vt) {
  VertexLCirculator vc( vt );
  std::cout << "(nf) center vt " << vt->id() << " boundary " << isBoundary(vt) << std::endl;
  std::shared_ptr<FaceL> fc = vc.beginFaceL();
  do {
    std::cout << "\t"; fc->print();
    fc = vc.nextFaceL();
  } while ( (fc != vc.firstFaceL()) && (fc != nullptr) );
  std::cout << std::endl;
}

inline void printNeighborVertices(std::shared_ptr<VertexL> ovt) {
  VertexLCirculator vc( ovt );
  std::cout << "(nv) center vt " << ovt->id() << " boundary " << isBoundary(ovt) << std::endl;
  std::shared_ptr<VertexL> vt = vc.beginVertexL();
  do {
    std::cout << "\t Vertex: " << vt->id() << std::endl;
    vt = vc.nextVertexL();
  } while ( (vt != vc.firstVertexL()) && (vt != nullptr) );
  std::cout << std::endl;
}

#endif // _MESHUTIL_HXX
