﻿////////////////////////////////////////////////////////////////////
//
// $Id: GLMesh.hxx 2025/07/19 15:04:16 kanai Exp 
//
//   Generic OpenGL Mesh draw class
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _GLMESH_HXX
#define _GLMESH_HXX 1

#include "envDep.h"

//#include "myGL.hxx"
#include <GL/gl.h>

#include "myEigen.hxx"
#include "GLMaterial.hxx"

class GLMesh {

protected:

  // material
  GLMaterial mtl_;
  GLMaterial mtl_point_;
  GLMaterial mtl_selected_;

  // color
  float pointColor_[3];
  float wireColor_[3];
  float boundaryColor_[3];
  float selectedColor_[3];

  // size
  float pointSize_;
  float wireSize_;
  float boundarySize_;

  bool isDrawShading_;
  bool isDrawWireframe_;
  bool isDrawPoint_;
  bool isDrawColor_;

  bool isDrawTexture_;

  bool isSmoothShading_;

public:

  GLMesh() { init(); };
  virtual ~GLMesh() = default;

  void init() {
    isDrawShading_ = true;
    isDrawColor_ = false;
    isDrawWireframe_ = false;
    isDrawPoint_ = false;
    isDrawTexture_ = false;
    isSmoothShading_ = false;
    mtl_.set(0);
    mtl_point_.set(13);
    mtl_selected_.set(10);
    setPointColor( .0f, .0f, 1.0f );
    setWireColor( .1f, .1f, .1f );
    setBoundaryColor( .0f, 1.0f, .0f );
    setSelectedColor( 1.0f, .0f, .0f );
    setPointSize( 3.0f );
    setWireSize( 1.0f );
    setBoundarySize( 2.0f );
  };

  GLMaterial& material() { return mtl_; };
  const GLMaterial& material() const { return mtl_; };

  void setMaterial( int no ) {
    mtl_.set( no );
  };

  void setMaterial( float* mtl ) {
    mtl_.set( mtl );
  };

  void setPointMaterial( float* mtl ) {
    mtl_point_.set( mtl );
  };

  float* getDiffuseColor() { return mtl_.getDiffuseColor(); };
  void setDiffuseColor( unsigned char r, unsigned char g,
                        unsigned char b, unsigned char a ) {
    mtl_.setDiffuseColor( (float) r / 255.0, (float) g / 255.0,
                          (float) b / 255.0, (float) a / 255.0 );
  };

  // color
  void setPointColor( Eigen::Vector3f& p ) {
    pointColor_[0] = p.x(); pointColor_[1] = p.y(); pointColor_[2] = p.z();
  };
  void setPointColor( float x, float y, float z ) {
    pointColor_[0] = x; pointColor_[1] = y; pointColor_[2] = z;
  };
  float* pointColor() { return pointColor_; };

  void setWireColor( unsigned char r, unsigned char g, unsigned char b ) {
    wireColor_[0] = (float) r / 255.0;
    wireColor_[1] = (float) g / 255.0;
    wireColor_[2] = (float) b / 255.0;
  };

  void setWireColor( Eigen::Vector3f& p ) {
    wireColor_[0] = p.x(); wireColor_[1] = p.y(); wireColor_[2] = p.z();
  };
  void setWireColor( float x, float y, float z ) {
    wireColor_[0] = x; wireColor_[1] = y; wireColor_[2] = z;
  };
  float* wireColor() { return wireColor_; };

  void setBoundaryColor( Eigen::Vector3f& p ) {
    boundaryColor_[0] = p.x(); boundaryColor_[1] = p.y(); boundaryColor_[2] = p.z();
  };
  void setBoundaryColor( float x, float y, float z ) {
    boundaryColor_[0] = x; boundaryColor_[1] = y; boundaryColor_[2] = z;
  };
  float* boundaryColor() { return boundaryColor_; };

  void setSelectedColor( Eigen::Vector3f& p ) {
    selectedColor_[0] = p.x(); selectedColor_[1] = p.y(); selectedColor_[2] = p.z();
  };
  void setSelectedColor( float x, float y, float z ) {
    selectedColor_[0] = x; selectedColor_[1] = y; selectedColor_[2] = z;
  };
  float* selectedColor() { return selectedColor_; };

  // size
  void setPointSize( float f ) { pointSize_ = f; };
  float pointSize() const { return pointSize_; };
  void setWireSize( float f ) { wireSize_ = f; };
  float wireSize() const { return wireSize_; };
  void setBoundarySize( float f ) { boundarySize_ = f; };
  float boundarySize() const { return boundarySize_; };

  void setIsDrawPoint( bool f ) { isDrawPoint_ = f; };
  void setIsDrawWireframe( bool f ) { isDrawWireframe_ = f; };
  void setIsDrawShading( bool f ) { isDrawShading_ = f; };
  void setIsDrawColor( bool f ) { isDrawColor_ = f; };
  void setIsDrawTexture( bool f ) { isDrawTexture_ = f; };
  void setIsSmoothShading( bool f ) {
    isSmoothShading_ = f;
    if ( isSmoothShading_ == true )
      {
        ::glShadeModel( GL_SMOOTH );
      }
    else
      {
        ::glShadeModel( GL_FLAT );
      }
  };
  bool isDrawPoint() const { return isDrawPoint_; };
  bool isDrawWireframe() const { return isDrawWireframe_; };
  bool isDrawShading() const { return isDrawShading_; };
  bool isDrawColor() const { return isDrawColor_; };
  bool isDrawTexture() const { return isDrawTexture_; };
  bool isSmoothShading() const { return isSmoothShading_; };

#if 0
  void drawSphere( Eigen::Vector3f& p, float radius, GLMaterial& glm ) {
    ::glEnable( GL_LIGHTING );

    glm.bind();

    GLUquadricObj   *qobj;
    GLint slices = 50,staks = 50;
    if ((qobj = gluNewQuadric()) != NULL) {
      ::glPushMatrix();
      ::glShadeModel( GL_SMOOTH );
      ::glTranslatef( p.x(), p.y(), p.z() );
      ::gluSphere(qobj, radius, slices, staks);
      ::glPopMatrix();
      ::gluDeleteQuadric(qobj);
    }

    ::glDisable( GL_LIGHTING );
  };
#endif

  virtual void draw() {
    ::glPushMatrix();
    if ( isDrawShading_ ) { drawShading(); }
    if ( isDrawPoint_ ) { drawPoint(); }
    if ( isDrawWireframe_ ) { drawWireframe(); }
    ::glPopMatrix();
  };

  virtual void drawShading() {};
  virtual void drawWireframe() {};
  virtual void drawPoint() {};

};

#endif // _GLMESH_HXX
