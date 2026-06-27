////////////////////////////////////////////////////////////////////////////////////
//
// $Id: GLKdTree3D.hxx 2026/06/26 23:01:00 kanai Exp
//
// Copyright (c) 2026 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////////////////////

#ifndef _GLKDTREE3D_HXX
#define _GLKDTREE3D_HXX 1

#include "envDep.h"
#include "myGL.hxx"

#include <algorithm>
#include <limits>
#include <vector>

#include "KdTree.hxx"
#include "GLShader.hxx"

class GLKdTree3D {
 public:
  GLKdTree3D()
      : kdtree_(nullptr),
        shader_(nullptr),
        splitVAO_(0),
        splitVBO_(0),
        leafVAO_(0),
        leafVBO_(0),
        num_split_lines_(0),
        num_leaf_lines_(0),
        draw_split_planes_(true),
        draw_leaf_boxes_(true),
        split_depth_filter_(-1) {}

  ~GLKdTree3D() {
    // deleteVAOVBO();
  }

  void deleteVAOVBO() {
    if (splitVAO_ != 0) {
      glDeleteVertexArrays(1, &splitVAO_);
      splitVAO_ = 0;
    }
    if (splitVBO_ != 0) {
      glDeleteBuffers(1, &splitVBO_);
      splitVBO_ = 0;
    }
    if (leafVAO_ != 0) {
      glDeleteVertexArrays(1, &leafVAO_);
      leafVAO_ = 0;
    }
    if (leafVBO_ != 0) {
      glDeleteBuffers(1, &leafVBO_);
      leafVBO_ = 0;
    }
  }

  void setKdTree(KdTree<3>& kdtree) { kdtree_ = &kdtree; }
  KdTree<3>& kdtree() { return *kdtree_; }

  bool empty() const { return (kdtree_ == nullptr); }

  void setShader(GLShader& shader) { shader_ = &shader; }
  const GLShader& shader() const { return *shader_; }

  void setDrawSplitPlanes(bool enabled) { draw_split_planes_ = enabled; }
  void setDrawLeafBoxes(bool enabled) { draw_leaf_boxes_ = enabled; }

  bool drawSplitPlanesEnabled() const { return draw_split_planes_; }
  bool drawLeafBoxesEnabled() const { return draw_leaf_boxes_; }

  int splitDepthFilter() const { return split_depth_filter_; }

  // -1: すべての深さ、0以上: 指定深さのみ分割面を描画
  void setSplitDepthFilter(int depth) {
    split_depth_filter_ = depth;
    rebuildBuffers();
  }

  void init3d(GLShader& shader) {
    setShader(shader);
    rebuildBuffers();
  }

  void rebuildBuffers() {
    if (empty()) return;

    deleteVAOVBO();

    std::vector<float> split_lines;
    std::vector<float> leaf_lines;

    Eigen::Vector3d bbmin, bbmax;
    computeRootBounds(bbmin, bbmax);

    buildGeometryRecursive(kdtree().root(), bbmin, bbmax, 0, split_lines,
                           leaf_lines);

    num_split_lines_ = static_cast<GLuint>(split_lines.size() / 6);
    num_leaf_lines_ = static_cast<GLuint>(leaf_lines.size() / 6);

    if (!split_lines.empty()) initLineVAO(splitVAO_, splitVBO_, split_lines);
    if (!leaf_lines.empty()) initLineVAO(leafVAO_, leafVBO_, leaf_lines);
  }

  void draw() {
    if (empty() || shader_ == nullptr) return;

    if (draw_split_planes_ && splitVAO_ != 0 && num_split_lines_ > 0) {
      drawLineBuffer(splitVAO_, num_split_lines_, 0.8f, 0.45f, 0.2f, 1.3f,
                     0.0002f);
    }
    if (draw_leaf_boxes_ && leafVAO_ != 0 && num_leaf_lines_ > 0) {
      drawLineBuffer(leafVAO_, num_leaf_lines_, 0.1f, 0.9f, 0.85f, 1.0f,
                     0.0004f);
    }
  }

 private:
  void computeRootBounds(Eigen::Vector3d& bbmin, Eigen::Vector3d& bbmax) {
    const auto& points = kdtree().points();

    bbmin = Eigen::Vector3d(std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::max());
    bbmax = Eigen::Vector3d(-std::numeric_limits<double>::max(),
                            -std::numeric_limits<double>::max(),
                            -std::numeric_limits<double>::max());

    for (const auto& p : points) {
      bbmin[0] = std::min(bbmin[0], p[0]);
      bbmin[1] = std::min(bbmin[1], p[1]);
      bbmin[2] = std::min(bbmin[2], p[2]);
      bbmax[0] = std::max(bbmax[0], p[0]);
      bbmax[1] = std::max(bbmax[1], p[1]);
      bbmax[2] = std::max(bbmax[2], p[2]);
    }

    for (int i = 0; i < 3; ++i) {
      if (std::fabs(bbmax[i] - bbmin[i]) < 1.0e-9) {
        bbmin[i] -= 1.0e-3;
        bbmax[i] += 1.0e-3;
      }
    }
  }

  void initLineVAO(GLuint& vao, GLuint& vbo, const std::vector<float>& buffer) {
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(float), buffer.data(),
                 GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);

    glBindVertexArray(0);
  }

  void drawLineBuffer(GLuint vao, GLuint line_count, float r, float g, float b,
                      float line_width, float depth_offset) {
    glUseProgram(shader().lines3dShaderProgram);
    glUniform3f(shader().lines3dLineColorLoc, r, g, b);
    glUniform1f(shader().lines3dLineWidthLoc, line_width);
    glUniform1f(shader().lines3dDepthOffsetLoc, depth_offset);

    glBindVertexArray(vao);
    glDrawArrays(GL_LINES, 0, line_count * 2);
    glBindVertexArray(0);
  }

  void addLine(std::vector<float>& buffer, const Eigen::Vector3d& a,
               const Eigen::Vector3d& b) {
    buffer.push_back(static_cast<float>(a[0]));
    buffer.push_back(static_cast<float>(a[1]));
    buffer.push_back(static_cast<float>(a[2]));
    buffer.push_back(static_cast<float>(b[0]));
    buffer.push_back(static_cast<float>(b[1]));
    buffer.push_back(static_cast<float>(b[2]));
  }

  void addBoxEdges(std::vector<float>& buffer, const Eigen::Vector3d& bbmin,
                   const Eigen::Vector3d& bbmax) {
    const Eigen::Vector3d p000(bbmin[0], bbmin[1], bbmin[2]);
    const Eigen::Vector3d p100(bbmax[0], bbmin[1], bbmin[2]);
    const Eigen::Vector3d p110(bbmax[0], bbmax[1], bbmin[2]);
    const Eigen::Vector3d p010(bbmin[0], bbmax[1], bbmin[2]);
    const Eigen::Vector3d p001(bbmin[0], bbmin[1], bbmax[2]);
    const Eigen::Vector3d p101(bbmax[0], bbmin[1], bbmax[2]);
    const Eigen::Vector3d p111(bbmax[0], bbmax[1], bbmax[2]);
    const Eigen::Vector3d p011(bbmin[0], bbmax[1], bbmax[2]);

    addLine(buffer, p000, p100);
    addLine(buffer, p100, p110);
    addLine(buffer, p110, p010);
    addLine(buffer, p010, p000);

    addLine(buffer, p001, p101);
    addLine(buffer, p101, p111);
    addLine(buffer, p111, p011);
    addLine(buffer, p011, p001);

    addLine(buffer, p000, p001);
    addLine(buffer, p100, p101);
    addLine(buffer, p110, p111);
    addLine(buffer, p010, p011);
  }

  void addSplitRectEdges(std::vector<float>& buffer, int axis,
                         double split_value, const Eigen::Vector3d& bbmin,
                         const Eigen::Vector3d& bbmax) {
    if (axis == 0) {
      const Eigen::Vector3d p0(split_value, bbmin[1], bbmin[2]);
      const Eigen::Vector3d p1(split_value, bbmax[1], bbmin[2]);
      const Eigen::Vector3d p2(split_value, bbmax[1], bbmax[2]);
      const Eigen::Vector3d p3(split_value, bbmin[1], bbmax[2]);
      addLine(buffer, p0, p1);
      addLine(buffer, p1, p2);
      addLine(buffer, p2, p3);
      addLine(buffer, p3, p0);
      return;
    }

    if (axis == 1) {
      const Eigen::Vector3d p0(bbmin[0], split_value, bbmin[2]);
      const Eigen::Vector3d p1(bbmax[0], split_value, bbmin[2]);
      const Eigen::Vector3d p2(bbmax[0], split_value, bbmax[2]);
      const Eigen::Vector3d p3(bbmin[0], split_value, bbmax[2]);
      addLine(buffer, p0, p1);
      addLine(buffer, p1, p2);
      addLine(buffer, p2, p3);
      addLine(buffer, p3, p0);
      return;
    }

    const Eigen::Vector3d p0(bbmin[0], bbmin[1], split_value);
    const Eigen::Vector3d p1(bbmax[0], bbmin[1], split_value);
    const Eigen::Vector3d p2(bbmax[0], bbmax[1], split_value);
    const Eigen::Vector3d p3(bbmin[0], bbmax[1], split_value);
    addLine(buffer, p0, p1);
    addLine(buffer, p1, p2);
    addLine(buffer, p2, p3);
    addLine(buffer, p3, p0);
  }

  void buildGeometryRecursive(KdNode* node, const Eigen::Vector3d& bbmin,
                              const Eigen::Vector3d& bbmax, int depth,
                              std::vector<float>& split_lines,
                              std::vector<float>& leaf_lines) {
    if (node == nullptr) return;

    const int axis = node->axis();
    const auto& p = kdtree().points()[node->idx()];
    const double split_value = p[axis];

    if (split_depth_filter_ < 0 || split_depth_filter_ == depth) {
      addSplitRectEdges(split_lines, axis, split_value, bbmin, bbmax);
    }

    Eigen::Vector3d left_min = bbmin;
    Eigen::Vector3d left_max = bbmax;
    Eigen::Vector3d right_min = bbmin;
    Eigen::Vector3d right_max = bbmax;

    left_max[axis] = split_value;
    right_min[axis] = split_value;

    KdNode* left = node->child(0);
    KdNode* right = node->child(1);

    if (left == nullptr && right == nullptr) {
      addBoxEdges(leaf_lines, bbmin, bbmax);
      return;
    }

    buildGeometryRecursive(left, left_min, left_max, depth + 1, split_lines,
                           leaf_lines);
    buildGeometryRecursive(right, right_min, right_max, depth + 1,
                           split_lines, leaf_lines);
  }

  KdTree<3>* kdtree_;
  GLShader* shader_;

  GLuint splitVAO_;
  GLuint splitVBO_;
  GLuint leafVAO_;
  GLuint leafVBO_;

  GLuint num_split_lines_;
  GLuint num_leaf_lines_;

  bool draw_split_planes_;
  bool draw_leaf_boxes_;
  int split_depth_filter_;
};

#endif  // _GLKDTREE3D_HXX
