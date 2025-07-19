////////////////////////////////////////////////////////////////////
//
// $Id: VertexL.hxx 2025/07/17 21:53:02 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _VERTEXL_HXX
#define _VERTEXL_HXX 1

#include "envDep.h"
#include "mydef.h"

#include <list>
#include <memory>
//using namespace std;

#include "myEigen.hxx"

#include "NodeL.hxx"
class HalfedgeL;

class VertexL : public NodeL {

public:

  VertexL() : NodeL(), halfedge_(nullptr) { init();};
  VertexL( int id ) : NodeL(id), halfedge_(nullptr) { init(); };
  ~VertexL(){};

  void init(){
    iter_ = std::list<std::shared_ptr<VertexL> >::iterator();
  };

  // point
  Eigen::Vector3d& point() { return point_; };
  void setPoint( Eigen::Vector3d& point ) { point_ = point; };
  void setPoint( double x, double y, double z ) {
    point_ << x, y, z;
  };

  // iter
  void setIter( std::list<std::shared_ptr<VertexL> >::iterator iter ) { iter_ = iter; };
  std::list<std::shared_ptr<VertexL> >::iterator iter() const { return iter_; };

  //
  // halfedge
  //
  std::shared_ptr<HalfedgeL> halfedge() const { return halfedge_; }
  void setHalfedge(std::shared_ptr<HalfedgeL> halfedge) { halfedge_ = halfedge; }

private:

  // 3D coord
  Eigen::Vector3d point_;

  // one of halfedges
  std::shared_ptr<HalfedgeL> halfedge_;

  // list iterator of MeshL
  std::list<std::shared_ptr<VertexL> >::iterator iter_;

};

#endif // _VERTEX_HXX
