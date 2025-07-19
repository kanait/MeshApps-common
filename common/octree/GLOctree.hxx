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

  GLOctree() : linesVAO_(0), linesVBO_(0) {};
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
};

#endif  // _GLOCTREE_HXX
