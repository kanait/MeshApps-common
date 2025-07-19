////////////////////////////////////////////////////////////////////
//
// $Id: NodeL.hxx 2021/06/05 14:19:10 kanai Exp $
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _NODEL_HXX
#define _NODEL_HXX 1

#include "envDep.h"
#include "mydef.h"

#include <iostream>

//class MeshL;

class NodeL {

public:

  NodeL() : id_(NULLID),selected_(false) { init(); };
  NodeL( int id ) : id_(id),selected_(false) { init(); };
  virtual ~NodeL(){};

  void init() {
    //mesh_ = NULL;
  };

  // id
  void setID( int id ) { id_ = id; };
  int id() const { return id_; };

  bool isSelected() const { return selected_; };
  void setSelected( bool f ) { selected_ = f; };

  // void setMesh( MeshL* mesh ) { mesh_ = mesh; };
  // MeshL* mesh() const { return mesh_; };

  friend std::ostream& operator <<( std::ostream& o, const NodeL& p ) {
    return o << p.id();
  };

private:

  int id_;
  bool selected_;

  //  MeshL* mesh_;

};

#endif //_NODEL_HXX
