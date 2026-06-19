////////////////////////////////////////////////////////////////////
//
// $Id: GLOctree.hxx 2025/07/15 02:06:07 kanai Exp 
//
// Copyright (c) 2024-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _GLOCTREE_HXX
#define _GLOCTREE_HXX 1

#include "envDep.h"
#include "myGL.hxx"

#include <memory>
#include "Octree.hxx"
#include "GLShader.hxx"

class GLOctree {

public:

  GLOctree() : linesVAO_(0), linesVBO_(0), rayVAO_(0), rayVBO_(0),
               hitVAO_(0), hitVBO_(0), has_hit_(false) {};
  ~GLOctree() {
    // deleteVAOVBO();
  };

  void deleteVAOVBO() {
    if (linesVAO_ != 0) {
      glDeleteVertexArrays(1, &linesVAO_);
      linesVAO_ = 0;
    }
    if (linesVBO_ != 0) {
      glDeleteBuffers(1, &linesVBO_);
      linesVBO_ = 0;
    }
    if (rayVAO_ != 0) {
      glDeleteVertexArrays(1, &rayVAO_);
      rayVAO_ = 0;
    }
    if (rayVBO_ != 0) {
      glDeleteBuffers(1, &rayVBO_);
      rayVBO_ = 0;
    }
    if (hitVAO_ != 0) {
      glDeleteVertexArrays(1, &hitVAO_);
      hitVAO_ = 0;
    }
    if (hitVBO_ != 0) {
      glDeleteBuffers(1, &hitVBO_);
      hitVBO_ = 0;
    }
  };

  void setOctree( std::shared_ptr<Octree> octree ) {
    octree_ = octree;
  };
  std::shared_ptr<Octree> octree() { return octree_; };

  void setShader(GLShader& shader) { shader_ = &shader; };
  const GLShader& shader() const { return *shader_; };

  void init3d(GLShader& shader) {
    setShader(shader);

    // convert kdtree points to OpenGL buffer for rendering
    std::vector<float> lines_buffer;
    num_lines_ = linesToBuffer(lines_buffer, octree_);
    std::cout << num_lines_ << " lines generated for Octree lines 3d rendering."
              << std::endl;

    // VAOs
    initLines3dVAO(lines_buffer);
  };

  void initRayIntersection(const Eigen::Vector3d& ray_pos,
                           const Eigen::Vector3d& ray_dir,
                           const Eigen::Vector3d& hit_point, bool has_hit) {
    std::vector<float> ray_buffer;
    ray_buffer.push_back(static_cast<float>(ray_pos.x()));
    ray_buffer.push_back(static_cast<float>(ray_pos.y()));
    ray_buffer.push_back(static_cast<float>(ray_pos.z()));
    ray_buffer.push_back(static_cast<float>(ray_pos.x() + 2.0 * ray_dir.x()));
    ray_buffer.push_back(static_cast<float>(ray_pos.y() + 2.0 * ray_dir.y()));
    ray_buffer.push_back(static_cast<float>(ray_pos.z() + 2.0 * ray_dir.z()));
    initRayVAO(ray_buffer);

    has_hit_ = has_hit;
    if (has_hit_) {
      std::vector<float> hit_buffer;
      hit_buffer.push_back(static_cast<float>(hit_point.x()));
      hit_buffer.push_back(static_cast<float>(hit_point.y()));
      hit_buffer.push_back(static_cast<float>(hit_point.z()));
      initHitVAO(hit_buffer);
    }
  };

  void initRayVAO(const std::vector<float>& ray_buffer) {
    if (rayVAO_ != 0) glDeleteVertexArrays(1, &rayVAO_);
    if (rayVBO_ != 0) glDeleteBuffers(1, &rayVBO_);

    glGenVertexArrays(1, &rayVAO_);
    glBindVertexArray(rayVAO_);

    glGenBuffers(1, &rayVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, rayVBO_);
    glBufferData(GL_ARRAY_BUFFER, ray_buffer.size() * sizeof(float),
                 ray_buffer.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);

    glBindVertexArray(0);
  };

  void initHitVAO(const std::vector<float>& hit_buffer) {
    if (hitVAO_ != 0) glDeleteVertexArrays(1, &hitVAO_);
    if (hitVBO_ != 0) glDeleteBuffers(1, &hitVBO_);

    glGenVertexArrays(1, &hitVAO_);
    glBindVertexArray(hitVAO_);

    glGenBuffers(1, &hitVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, hitVBO_);
    glBufferData(GL_ARRAY_BUFFER, hit_buffer.size() * sizeof(float),
                 hit_buffer.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);

    glBindVertexArray(0);
  };

  void drawRayIntersection() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(shader().lines3dShaderProgram);
    glUniform1f(shader().lines3dLineWidthLoc, 2.0f);
    glUniform3f(shader().lines3dLineColorLoc, 0.0f, 0.0f, 0.0f);
    glUniform1f(shader().lines3dDepthOffsetLoc, 0.0f);
    glBindVertexArray(rayVAO_);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);

    if (has_hit_) {
      glUseProgram(shader().points3dShaderProgram);
      glUniform1f(shader().points3dPointSizeLoc, 8.0f);
      glUniform3f(shader().points3dPointColorLoc, 1.0f, 0.0f, 0.0f);
      glUniform1f(shader().points3dDepthOffsetLoc, 0.0f);
      glBindVertexArray(hitVAO_);
      glDrawArrays(GL_POINTS, 0, 1);
      glBindVertexArray(0);
    }

    glDisable(GL_BLEND);
  };

  void initLines3dVAO( std::vector<float>& lines_buffer ) {
    glGenVertexArrays(1, &linesVAO_);
    glBindVertexArray(linesVAO_);

    GLuint vbo = 0;
    glGenBuffers(1, &linesVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, linesVBO_);
    glBufferData(GL_ARRAY_BUFFER, lines_buffer.size() * sizeof(float), lines_buffer.data(),
                 GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,  // location = 0
                          3,  // 3 floats per vertex
                          GL_FLOAT, GL_FALSE,
                          3 * sizeof(float),  // stride: 1 vertex = 3 floats
                          (void*)0); // offset

    glBindVertexArray(0);
  };

  void drawOctree() {
    glUseProgram(shader().lines3dShaderProgram);
    glUniform3f(shader().lines3dLineColorLoc, 0.2f, 0.8f, 0.2f);  // RGB
    glUniform1f(shader().lines3dDepthOffsetLoc, 0.0f); 

    glBindVertexArray(linesVAO_);
    glDrawArrays(GL_LINES, 0, num_lines_ * 2);  // 頂点数を記載する
    glBindVertexArray(0);
  };

  int linesToBuffer(std::vector<float>& lines_buffer,
                    std::shared_ptr<Octree> node) {
    if (node == nullptr) return 0;

    // 最大レベルを超えたら格納しない
    if (node->level() > MAX_LEVEL) return 0;

    // store vertices to line_buffer
    int num_lines = 0;
    Eigen::Vector3f minf = node->getBBmin().cast<float>();
    Eigen::Vector3f maxf = node->getBBmax().cast<float>();

    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(minf.z());
    num_lines++;

    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(minf.z());
    num_lines++;

    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(minf.z());
    num_lines++;

    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(minf.z());
    num_lines++;

    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(maxf.z());
    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(maxf.z());
    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(maxf.z());
    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(maxf.z());
    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(minf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(maxf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(minf.z());
    lines_buffer.push_back(maxf.x());
    lines_buffer.push_back(minf.y());
    lines_buffer.push_back(maxf.z());
    num_lines++;

    // apply to children
    for (int i = 0; i < 8; ++i)
      num_lines += linesToBuffer(lines_buffer, node->child(i));

    return num_lines;
  };

private:

  std::shared_ptr<Octree> octree_;
  GLShader* shader_;
  GLuint linesVAO_;
  GLuint linesVBO_;
  GLuint num_lines_;
  GLuint rayVAO_;
  GLuint rayVBO_;
  GLuint hitVAO_;
  GLuint hitVBO_;
  bool has_hit_;
};

#endif  // _GLOCTREE_HXX
