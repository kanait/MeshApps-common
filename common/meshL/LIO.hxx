////////////////////////////////////////////////////////////////////
//
// $Id: LIO.hxx 2022/04/02 01:14:37 kanai Exp $
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _LIO_HXX
#define _LIO_HXX 1

#include "MeshL.hxx"

class LIO {

  MeshL* mesh_;

  bool saveTexcoord_;
  bool saveNormal_;
  bool saveBLoop_;

public:

  LIO() { init(); };
  LIO( MeshL& mesh ) { init(); setMesh( mesh ); };
  virtual ~LIO() {};
  
  void init() {
    mesh_ = nullptr;
    saveTexcoord_ = false;
    saveNormal_ = false;
    saveBLoop_ = false;
  };
  
  MeshL& mesh() { return *mesh_; };
  bool empty() const { return ( mesh_ == nullptr ) ? true : false; };
  void setMesh( MeshL& mesh ) { mesh_ = &mesh; };

  bool isSaveTexcoord() const { return saveTexcoord_; };
  void setSaveTexcoord( bool f ) { saveTexcoord_ = f; };

  bool isSaveNormal() const { return saveNormal_; };
  void setSaveNormal( bool f ) { saveNormal_ = f; };

  bool isSaveBLoop() const { return saveBLoop_; };
  void setSaveBLoop( bool f ) { saveBLoop_ = f; };
};

#endif // _LIO_HXX
