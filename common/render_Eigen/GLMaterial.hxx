////////////////////////////////////////////////////////////////////
//
// $Id: GLMaterial.hxx 2025/07/19 15:03:47 kanai Exp 
//
//   Material setting class for OpenGL based on Eigen
//
// Copyright (c) 2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _GLMATERIAL_HXX
#define _GLMATERIAL_HXX 1

#include "envDep.h"
#include <GL/gl.h>
//#include "myGL.hxx"
#include "mtldata.h"

#include "myEigen.hxx"

#define DEFAULT_MAT 0

static GLfloat diffuseColors1[] = {
  .8f, .8f, .8f, 1.0f, // default
  .8f, .2f, .2f, 1.0f, // red
  .2f, .8f, .2f, 1.0f, // green
  .2f, .2f, .8f, 1.0f, // blue
  .8f, .8f, .2f, 1.0f, // yellow
  .8f, .2f, .8f, 1.0f, // magenta
  .2f, .8f, .8f, 1.0f, // cyan
  .2f, .2f, .2f, 1.0f, // black
  .4f, .4f, .4f, 1.0f, 
  .8f, .4f, .4f, 1.0f, 
  .4f, .8f, .4f, 1.0f, 
  .4f, .4f, .8f, 1.0f, 
  .8f, .8f, .4f, 1.0f, 
  .8f, .4f, .8f, 1.0f, 
  .4f, .8f, .8f, 1.0f
};

class GLMaterial {
public:
  GLMaterial() { init(); }

  GLMaterial(float* mtl) { set(mtl); }

  GLMaterial(float* ambient, float* diffuse, float* specular,
             float* emission, float shininess) {
    set(ambient, diffuse, specular, emission, shininess);
  }

  ~GLMaterial() = default;

  void init() {
    set(DEFAULT_MAT);
  }

  void set(float* ambient, float* diffuse, float* specular,
           float* emission, float shininess) {
    ambient_  = Eigen::Map<Eigen::Vector4f>(ambient);
    diffuse_  = Eigen::Map<Eigen::Vector4f>(diffuse);
    specular_ = Eigen::Map<Eigen::Vector4f>(specular);
    emission_ = Eigen::Map<Eigen::Vector4f>(emission);
    shininess_ = shininess;
  }

  void set(float* mtl) {
    ambient_  = Eigen::Map<Eigen::Vector4f>(&mtl[0]);
    diffuse_  = Eigen::Map<Eigen::Vector4f>(&mtl[4]);
    emission_ = Eigen::Map<Eigen::Vector4f>(&mtl[8]);
    specular_ = Eigen::Map<Eigen::Vector4f>(&mtl[12]);
    shininess_ = mtl[16];
  }

  void set(int no) {
    int id = no * NUM_MTL_ITEMS;
    ambient_  = Eigen::Map<const Eigen::Vector4f>(&mtlall[id + 0]);
    diffuse_  = Eigen::Map<const Eigen::Vector4f>(&mtlall[id + 4]);
    emission_ = Eigen::Map<const Eigen::Vector4f>(&mtlall[id + 8]);
    specular_ = Eigen::Map<const Eigen::Vector4f>(&mtlall[id + 12]);
    shininess_ = mtlall[id + 16];
  }

  Eigen::Vector4f ambient() const  { return ambient_;  };
  Eigen::Vector4f diffuse() const  { return diffuse_;  };
  Eigen::Vector4f specular() const { return specular_; };
  Eigen::Vector4f emission() const { return emission_; };
  Eigen::Vector3f ambient3() const  { return ambient_.head<3>();  };
  Eigen::Vector3f diffuse3() const  { return diffuse_.head<3>();  };
  Eigen::Vector3f specular3() const { return specular_.head<3>(); };
  Eigen::Vector3f emission3() const { return emission_.head<3>(); };
  float shininess() const          { return shininess_; }

  float* getDiffuseColor() { return diffuse_.data(); }

  void setDiffuseColor(float r, float g, float b, float a) {
    diffuse_ = Eigen::Vector4f(r, g, b, a);
    ::glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_.data());
  }

  void setDiffuseColor(int n) {
    int i = (n > 14) ? 14 : n;
    diffuse_ = Eigen::Map<const Eigen::Vector4f>(&diffuseColors1[4 * i]);
    ::glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_.data());
  }

  void bind() {
    ::glMaterialfv(GL_FRONT, GL_AMBIENT,   ambient_.data());
    ::glMaterialfv(GL_FRONT, GL_DIFFUSE,   diffuse_.data());
    ::glMaterialfv(GL_FRONT, GL_SPECULAR,  specular_.data());
    ::glMaterialfv(GL_FRONT, GL_EMISSION,  emission_.data());
    ::glMaterialfv(GL_FRONT, GL_SHININESS, &shininess_);
  }

private:
  Eigen::Vector4f ambient_;
  Eigen::Vector4f diffuse_;
  Eigen::Vector4f emission_;
  Eigen::Vector4f specular_;
  float shininess_;
};

#endif // _GLMATERIAL_HXX
