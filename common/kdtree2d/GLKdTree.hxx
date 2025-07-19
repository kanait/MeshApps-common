////////////////////////////////////////////////////////////////////////////////////
//
// $Id: GLKdTree.hxx 2025/07/15 14:28:08 kanai Exp 
//
// Copyright (c) 2024-2025 by Takashi Kanai. All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////////

#ifndef _GLKDTREE_HXX
#define _GLKDTREE_HXX 1

#include "envDep.h"
#include "myGL.hxx"

#include "KdTree.hxx"
#include "GLShader.hxx"

template<int N>
class GLKdTree {

public:

  GLKdTree( int width, int height ) : kdtree_(nullptr),shader_(nullptr) {
    width_ = (float) width;
    height_ = (float) height;
    pointsVAO_=0;
    pointsVBO_=0;
    linesVAO_=0;
    linesVBO_=0;
    num_points_=0;
    num_lines_=0;
  };
  ~GLKdTree() {
    // deleteVAOVBO();
  };

  void deleteVAOVBO() {
    if (glIsVertexArray(pointsVAO_)) glDeleteVertexArrays(1, &pointsVAO_);
    if (glIsBuffer(pointsVBO_)) glDeleteBuffers(1, &pointsVBO_);
    if (glIsVertexArray(linesVAO_)) glDeleteVertexArrays(1, &linesVAO_);
    if (glIsBuffer(linesVBO_)) glDeleteBuffers(1, &linesVBO_);
  };

  void setKdTree( KdTree<N>& kdtree ) { kdtree_ = &kdtree; };
  KdTree<N>& kdtree() { return *kdtree_; };

  bool empty() const { return ( kdtree_ != nullptr ) ? false : true; };

  void setShader(GLShader& shader) { shader_ = &shader; };
  const GLShader& shader() const { return *shader_; };

  void init2d(GLShader& shader) {
    // std::cout << "Initializing GLKdTree 2D..." << std::endl;
    setShader(shader);

    // convert kdtree points to OpenGL buffer for rendering
    std::vector<float> points_buffer;
    convertPointsToOpenGLBuffer(kdtree().points(), points_buffer);
    num_points_ = kdtree().points().size();
    // std::cout << "Converted " << kdtree().points().size() << " points to OpenGL buffer" << std::endl;

    std::vector<float> lines_buffer;
    num_lines_ = convertKdTreeLinesToOpenGLBuffer(lines_buffer);
    std::cout << num_lines_ << " lines generated for KdTree rendering." << std::endl;

    // VAOs
    initPoints2dVAO(points_buffer);
    initLines2dVAO(lines_buffer);
  };

  // lines 2d VAOとVBOの設定
  void initLines2dVAO(std::vector<float>& lines_buffer) {
    glGenVertexArrays(1, &linesVAO_);
    glBindVertexArray(linesVAO_);

    glGenBuffers(1, &linesVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, linesVBO_);
    glBufferData(GL_ARRAY_BUFFER, lines_buffer.size() * sizeof(float), lines_buffer.data(),
                 GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,  // location = 0
                          2,  // vec2 = 2 floats per vertex
                          GL_FLOAT, GL_FALSE,
                          2 * sizeof(float),  // stride
                          (void*)0);

    glBindVertexArray(0);
  };

  // points 2d VAOとVBOの設定
  void initPoints2dVAO(std::vector<float>& points_buffer) {
    glGenVertexArrays(1, &(pointsVAO_));
    glBindVertexArray(pointsVAO_);

    glGenBuffers(1, &pointsVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO_);
    glBufferData(GL_ARRAY_BUFFER, points_buffer.size() * sizeof(float),
                 points_buffer.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glEnable(GL_PROGRAM_POINT_SIZE);  // シェーダーで点サイズを制御可能にする
  };

  void draw2d() {
    if ( empty() ) return;
    drawPoints2d();
    drawLines2d();
  };

  void drawLines2d() {
    if ( empty() ) return;
    glUseProgram(shader().lines2dShaderProgram);

    // uniform変数を設定
    glUniform2f(shader().lines2dScreenSizeLoc, width_, height_ );
    glUniform1f(shader().lines2dLineWidthLoc, 1.0f);  // ピクセル単位の太さ
    glUniform3f(shader().lines2dLineColorLoc, 0.2f, 0.8f, 0.2f);  // RGB
    
    glBindVertexArray(linesVAO_);
    glDrawArrays(GL_LINES, 0, num_lines_ * 2);  // 頂点数を記載する
    glBindVertexArray(0);
  };

  void drawPoints2d() {
    if ( empty() ) return;
    glUseProgram(shader().points2dShaderProgram);
    
    // uniform変数を設定
    glUniform2f(shader().points2dScreenSizeLoc, width_, height_);
    glUniform1f(shader().points2dPointSizeLoc, 1.0f);  // ピクセル単位の太さ
    glUniform3f(shader().points2dPointColorLoc, 1.0f, 0.0f, 0.0f);  // RGB
    
    glBindVertexArray(pointsVAO_);
    glDrawArrays(GL_POINTS, 0, num_points_);
    glBindVertexArray(0);
  };

  void drawClosePointSets2d(std::vector<int>& idx ) {

    if ( empty() ) return;

    ::glPointSize( 3.0f );
    ::glColor3f( 1.0f, .0f, .0f );

    ::glBegin( GL_POINTS );
    for ( int i = 0; i < idx.size() ; ++i ) {
      Eigen::Vector<double,N>& p = kdtree().points()[idx[i]];
      ::glVertex2f( p[0], p[1] );
    }
    ::glEnd();
  };

  void convertPointsToOpenGLBuffer(const std::vector<Eigen::Vector<double, N>>& points,
                                   std::vector<float>& points_buffer) {
    points_buffer.clear();
    points_buffer.reserve(points.size() * N); // 必要な容量を確保

    for (const auto& pt : points) {
      // 元の座標（0-800の範囲）をそのまま渡す
      points_buffer.push_back(static_cast<float>(pt[0]));
      points_buffer.push_back(static_cast<float>(pt[1]));
    }
  };

  int convertKdTreeLinesToOpenGLBuffer(std::vector<float>& lines_buffer) {
    if ( empty() ) return 0;

    float bbMin[2], bbMax[2];
    bbMin[0] = .0f;
    bbMin[1] = .0f;
    bbMax[0] = width_;
    bbMax[1] = height_;

    auto points = kdtree().points();
    return linesToBuffer( lines_buffer, points,
                          kdtree().root(), 0, points.size()-1, 0, bbMin, bbMax );

  };

  int linesToBuffer(
    std::vector<float>& lines_buffer,
    std::vector<Eigen::Vector<double,N> >& points,
    KdNode* node, int left, int right, int splitDimension,
    float bbMin[], float bbMax[]) {
    if (node == nullptr) return 0;

    // Calculate the index of this node:
    int mid = (left + right) >> 1;

    // Calculate this node's bounding box:
    Eigen::Vector<double, N>& p = points[node->idx()];

    int num_lines = 0;
    if (splitDimension == 1) {  // x
      lines_buffer.push_back(bbMin[0]);
      lines_buffer.push_back(p[1]);
      lines_buffer.push_back(bbMax[0]);
      lines_buffer.push_back(p[1]);
      num_lines++;
    } else {
      lines_buffer.push_back(p[0]);
      lines_buffer.push_back(bbMin[1]);
      lines_buffer.push_back(p[0]);
      lines_buffer.push_back(bbMax[1]);
      num_lines++;
    }

    int childSplitDimension = (splitDimension + 1) % N;

    float childMin[2], childMax[2];
    if (left < mid) {
      if (childSplitDimension == 0) {
        childMin[0] = bbMin[0];
        childMax[0] = bbMax[0];
        childMin[1] = bbMin[1];
        childMax[1] = p[1];
      } else {
        childMin[0] = bbMin[0];
        childMax[0] = p[0];
        childMin[1] = bbMin[1];
        childMax[1] = bbMax[1];
      }
      num_lines += linesToBuffer(lines_buffer, points, node->child(0), left, mid - 1,
                                 childSplitDimension, childMin, childMax);
    }

    if (right > mid) {
      if (childSplitDimension == 0) {
        childMin[0] = bbMin[0];
        childMax[0] = bbMax[0];
        childMin[1] = p[1];
        childMax[1] = bbMax[1];
      } else {
        childMin[0] = p[0];
        childMax[0] = bbMax[0];
        childMin[1] = bbMin[1];
        childMax[1] = bbMax[1];
      }
      num_lines += linesToBuffer(lines_buffer, points, node->child(1), mid + 1, right,
                                 childSplitDimension, childMin, childMax);
    }
    return num_lines;
  };

private:

  KdTree<N>* kdtree_;
  float width_;
  float height_;

  GLShader* shader_;
  GLuint pointsVAO_;
  GLuint pointsVBO_;
  GLuint linesVAO_;
  GLuint linesVBO_;
  GLuint num_points_;
  GLuint num_lines_;

};

#endif // _GLKDTREE_HXX
