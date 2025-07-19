////////////////////////////////////////////////////////////////////
//
// $Id: TexcoordL.hxx 2025/07/17 21:58:06 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _TEXCOORDL_HXX
#define _TEXCOORDL_HXX 1

#include "envDep.h"
#include "mydef.h"

#include <list>
#include <memory>
//using namespace std;

#include "myEigen.hxx"

#include "NodeL.hxx"

class TexcoordL : public NodeL {

public:

  TexcoordL() : NodeL() { init(); };
  TexcoordL( int id ) : NodeL(id) { init(); };
  ~TexcoordL(){};

  void init() {
    iter_ = std::list<std::shared_ptr<TexcoordL> >::iterator();
  };

  // vector
  Eigen::Vector3d& point() { return point_; };
  void setPoint( Eigen::Vector3d& p ) { point_ = p; };
  void setPoint( double x, double y, double z ) {
    point_ << x, y, z;
  };

  // iter
  void setIter( std::list<std::shared_ptr<TexcoordL> >::iterator iter ) { iter_ = iter; };
  std::list<std::shared_ptr<TexcoordL> >::iterator iter() { return iter_; };

private:

  // texture coordinate
  Eigen::Vector3d point_;

  // list iterator of MeshL
  std::list<std::shared_ptr<TexcoordL> >::iterator iter_;

};

#endif // _TEXCOORDL_HXX

