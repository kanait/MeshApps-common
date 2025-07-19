////////////////////////////////////////////////////////////////////
//
// $Id: NormalL.hxx 2025/07/17 21:57:47 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _NORMALL_HXX
#define _NORMALL_HXX 1

#include <list>
#include <memory>
//using namespace std;

#include "myEigen.hxx"

#include "NodeL.hxx"

class NormalL : public NodeL {

public:

  NormalL() : NodeL() { init(); };
  NormalL( int id ) : NodeL(id) { init(); };
  ~NormalL(){};

  void init() {
    iter_ = std::list<std::shared_ptr<NormalL> >::iterator();
  };

  // vector
  Eigen::Vector3d& point() { return point_; };
  void setPoint( Eigen::Vector3d& p ) { point_ = p; };
  void setPoint( double x, double y, double z ) {
    point_ << x, y, z;
  };

  // iter
  void setIter( std::list<std::shared_ptr<NormalL> >::iterator iter ) { iter_ = iter; };
  std::list<std::shared_ptr<NormalL> >::iterator iter() { return iter_; };

private:

  // normal vector
  Eigen::Vector3d point_;

  // list iterator of MeshL
  std::list<std::shared_ptr<NormalL> >::iterator iter_;
};

#endif // _NORMALL_HXX


