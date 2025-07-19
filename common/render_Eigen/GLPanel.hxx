////////////////////////////////////////////////////////////////////
//
// $Id: GLPanel.hxx 2025/07/19 15:29:36 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _GLPANEL_HXX
#define _GLPANEL_HXX 1

#include "envDep.h"
#include "mydef.h"

#include <cmath>
#include <iostream>
#include <array>
#include <stdexcept>
#include <string>
#include <string_view>
#include <fstream>
#include <vector>

#include "myGL.hxx"
#include "myEigen.hxx"

#include "Arcball.hxx"
#include "GLShader.hxx"
#include "GLMaterial.hxx"

// Custom exception classes for better error handling
class GLPanelException : public std::runtime_error {
 public:
  explicit GLPanelException(const std::string& message)
      : std::runtime_error(message) {}
};

class ShaderException : public GLPanelException {
 public:
  explicit ShaderException(const std::string& message)
      : GLPanelException("Shader error: " + message) {}
};

class OpenGLException : public GLPanelException {
 public:
  explicit OpenGLException(const std::string& message)
      : GLPanelException("OpenGL error: " + message) {}
};

class GLPanel {
 public:
  // Constants for magic numbers
  static constexpr size_t NUM_SHADER_PROGRAMS = 5;
  static constexpr size_t NUM_LIGHTS = 4;
  static constexpr size_t BACKGROUND_COLOR_SIZE = 3;
  static constexpr size_t MATRIX_SIZE = 16;
  static constexpr float DEFAULT_FOV = 30.0f;
  static constexpr float DEFAULT_NEAR_PLANE = 0.01f;
  static constexpr float DEFAULT_FAR_PLANE = 100000.0f;
  static constexpr float DEFAULT_VIEW_DISTANCE = 3.0f;
  static constexpr int PHONG_SHADING_INDEX = 0;
  static constexpr int GOURAND_SHADING_INDEX = 1;
  static constexpr int WIREFRAME_INDEX = 2;
  static constexpr int PHONG_TEXTURE_INDEX = 3;
  static constexpr int COLOR_RENDERING_INDEX = 4;

  // Mathematical constants
  static constexpr float FOV_TO_RAD = M_PI / 180.0f;
  static constexpr float HALF_FOV_TO_RAD = M_PI / 360.0f;
  static constexpr float MIN_SCALE_2D = 0.01f;
  static constexpr float ZOOM_SENSITIVITY = 0.1f;

  GLPanel() : gradVAO_(0), gradVBO_(0) {};
  ~GLPanel(){
    //delete3dShaders();
    //delete2dShaders();
    //deleteVAOVBO();
  };

  void init(const int w, const int h) {
    // Parameter validation
    if (w <= 0 || h <= 0) {
      throw GLPanelException("Invalid viewport dimensions: " +
                             std::to_string(w) + "x" + std::to_string(h));
    }

    std::cout << "Initializing GLPanel with dimensions: " << w << "x" << h
              << std::endl;

    bgrgb_.fill(1.0f);
    setW(w);
    setH(h);
    initViewParameters(w, h);

    transform_flags_.rotate = false;
    transform_flags_.move = false;
    transform_flags_.zoom = false;
    display_flags_.gradient_background = true;

  }

  void initViewParameters(const int w, const int h) {
    // set projection parameters
    projection_.fov = DEFAULT_FOV;
    viewport_.aspect = static_cast<float>(w) / static_cast<float>(h);
    projection_.near_plane = DEFAULT_NEAR_PLANE;
    projection_.far_plane = DEFAULT_FAR_PLANE;

    // view point, view vector
    view_params_.view_point << 0.0f, 0.0f, DEFAULT_VIEW_DISTANCE;
    view_params_.view_vector << 0.0f, 0.0f, -DEFAULT_VIEW_DISTANCE;
    view_params_.look_point << 0.0f, 0.0f, 0.0f;

    // for Arcball
    manip_.init();
    manip_.setHalfWHL(static_cast<int>(w / 2.0), static_cast<int>(h / 2.0));
  };

  // GLAD is initialized in initGL() and initGL2d() with gladLoadGL()
  // This function is kept for compatibility but does nothing
  bool initGLAD() {
    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
      std::cerr << "Failed to initialize GLAD" << std::endl;
      return false;
    }
    return true;
  };

  // Initialize shaders
  void initShader() {

    // Shaders for Phong Shading
    shader_.phongShaderProgram = createShaderProgram( vertex_shader_Phong_source33,
                                                      nullptr,
                                                      fragment_shader_Phong_source33 );
    
    // Phongシェーディング用シェーダに入れる変数の情報を取得
    shader_.projectionLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "projection");
    shader_.modelviewLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "modelview");
    shader_.normalmatrixLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "normalMatrix");

    for (int i = 0; i < NUM_LIGHTS; ++i) {
      std::string name = "light_position[" + std::to_string(i) + "]";
      shader_.lightpositionLoc[i] =
          glGetUniformLocation(shader_.phongShaderProgram, name.c_str());
      name = "light_enabled[" + std::to_string(i) + "]";
      shader_.lightenabledLoc[i] =
          glGetUniformLocation(shader_.phongShaderProgram, name.c_str());
    }
    shader_.ambientcolorLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "ambient_color");
    shader_.diffusecolorLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "diffuse_color");
    shader_.emissioncolorLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "emission_color");
    shader_.specularcolorLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "specular_color");
    shader_.shininessLoc =
        glGetUniformLocation(shader_.phongShaderProgram, "shininess");

    // Shaders for wireframe rendering
    shader_.wireframeShaderProgram = createShaderProgram( vertex_wireframe_source33, nullptr,
                                                          fragment_wireframe_source33 );

    // ワイヤフレーム用シェーダに入れる変数の情報を取得
    shader_.wireframemodelviewLoc =
        glGetUniformLocation(shader_.wireframeShaderProgram, "modelview");
    shader_.wireframeprojectionLoc =
        glGetUniformLocation(shader_.wireframeShaderProgram, "projection");

    // Shaders for lines 3d rendering
    shader_.lines3dShaderProgram = createShaderProgram( vertex_lines3d_source33,
                                                        geometry_lines3d_source33,
                                                        fragment_lines3d_source33 );

    // lines 3d 用の uniform の情報を取得
    shader_.lines3dModelViewLoc = glGetUniformLocation(shader_.lines3dShaderProgram, "modelview");
    shader_.lines3dProjectionLoc = glGetUniformLocation(shader_.lines3dShaderProgram, "projection");
    shader_.lines3dViewportSizeLoc = glGetUniformLocation(shader_.lines3dShaderProgram, "viewport_size");
    shader_.lines3dLineWidthLoc = glGetUniformLocation(shader_.lines3dShaderProgram, "line_width");
    shader_.lines3dAspectLoc = glGetUniformLocation(shader_.lines3dShaderProgram, "aspect");
    shader_.lines3dLineColorLoc = glGetUniformLocation(shader_.lines3dShaderProgram, "line_color");
    shader_.lines3dDepthOffsetLoc = glGetUniformLocation(shader_.lines3dShaderProgram, "depth_offset");

    // Shaders for gradient background
    shader_.gradShaderProgram = createShaderProgram( gradVertShaderSrc, nullptr,
                                                     gradFragShaderSrc );
    
    // VAO, VBO for gradient background
    float gradVertices[] = {
        // pos         // color
        -1.0f, -1.0f, 0.0f, 0.0f, 0.1f,  // 左下
        1.0f,  -1.0f, 0.0f, 0.0f, 0.1f,  // 右下
        -1.0f, 1.0f,  0.4f, 0.4f, 1.0f,  // 左上
        1.0f,  1.0f,  0.4f, 0.4f, 1.0f   // 右上
    };

    glGenVertexArrays(1, &(gradVAO_));
    glBindVertexArray(gradVAO_);
    
    glGenBuffers(1, &gradVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, gradVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gradVertices), gradVertices,
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  };

  const GLShader& shader() const { return shader_; };
  GLShader& shader() { return shader_; };

  //
  // 2D Functions
  //

  void initGL2d() {

    // GLAD is initialized
    initGLAD();
    
    ::glDisable(GL_ALPHA_TEST);
    ::glDisable(GL_BLEND);
    ::glDisable(GL_DEPTH_TEST);
    ::glDisable(GL_LIGHTING);
    ::glDisable(GL_TEXTURE_1D);
    ::glDisable(GL_TEXTURE_2D);
    ::glDisable(GL_POLYGON_OFFSET_FILL);
    //   ::glShadeModel( GL_FLAT );

    const GLubyte* version = glGetString(GL_VERSION);
    const GLubyte* renderer = glGetString(GL_RENDERER);
    std::cout << "OpenGL Version: " << version << std::endl;
    std::cout << "OpenGL Renderer: " << renderer << std::endl;

    setIsGradientBackground(false);
  };

  void initShader2d() {
    // shaders for point 2d rendering
    shader_.points2dShaderProgram = createShaderProgram( vertex_points2d_source33,
                                                         geometry_points2d_source33,
                                                         fragment_points2d_source33 );
    shader_.points2dPointSizeLoc =
      glGetUniformLocation(shader_.points2dShaderProgram, "pointSize");
    shader_.points2dScreenSizeLoc =
      glGetUniformLocation(shader_.points2dShaderProgram, "screenSize");
    shader_.points2dPointColorLoc =
      glGetUniformLocation(shader_.points2dShaderProgram, "pointColor");

    // shaders for line 2d rendering
    shader_.lines2dShaderProgram = createShaderProgram( vertex_lines2d_source33,
                                                        geometry_lines2d_source33,
                                                        fragment_lines2d_source33 );

    // 各 uniform location を取得
    shader_.lines2dScreenSizeLoc = glGetUniformLocation(
      shader_.lines2dShaderProgram, "viewport_size");
    shader_.lines2dLineWidthLoc  = glGetUniformLocation(
      shader_.lines2dShaderProgram, "line_width");
    shader_.lines2dLineColorLoc  = glGetUniformLocation(
      shader_.lines2dShaderProgram, "line_color");

    // std::cout << "2D shaders initialization completed" << std::endl;
  };

  GLuint compileShader(GLenum type, const char* src) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      char log[512];
      glGetShaderInfoLog(shader, 512, nullptr, log);
      std::cerr << "Shader compile error: " << log << std::endl;
    }
    return shader;
  };

  GLuint createShaderProgram(const char* vsrc, const char* gsrc,
                             const char* fsrc) {
    GLuint vs = compileShader(GL_VERTEX_SHADER, vsrc);
    GLuint gs;
    if (gsrc) gs = compileShader(GL_GEOMETRY_SHADER, gsrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsrc);

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    if (gsrc) glAttachShader(program, gs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
      char log[512];
      glGetProgramInfoLog(program, 512, nullptr, log);
      std::cerr << "Program link error: " << log << std::endl;
    }

    glDeleteShader(vs);
    if (gsrc) glDeleteShader(gs);
    glDeleteShader(fs);
    return program;
  };

  void delete3dShaders() {
    if (shader_.phongShaderProgram) {
      glDeleteProgram(shader_.phongShaderProgram);
    }
    if (shader_.wireframeShaderProgram) {
      glDeleteProgram(shader_.wireframeShaderProgram);
    }
    if (shader_.gradShaderProgram) {
      glDeleteProgram(shader_.gradShaderProgram);
    }
    if (shader_.lines3dShaderProgram) {
      glDeleteProgram(shader_.lines3dShaderProgram);
    }
  };

  void delete2dShaders() {
    if (shader_.points2dShaderProgram) {
      glDeleteProgram(shader_.points2dShaderProgram);
    }
    if (shader_.lines2dShaderProgram) {
      glDeleteProgram(shader_.lines2dShaderProgram);
    }
  };

  void deleteVAOVBO() {
    if (glIsVertexArray(gradVAO_)) glDeleteVertexArrays(1, &gradVAO_);
    if (glIsBuffer(gradVBO_)) glDeleteBuffers(1, &gradVBO_);
    gradVAO_ = 0;
    gradVBO_ = 0;
  };
  
  void clear2d() { clear2d(w(), h()); };
  void clear2d(int w, int h) {
    ::glViewport(0, 0, w, h);
    ::glClearColor(bgrgb_[0], bgrgb_[1], bgrgb_[2], 1.0f);
    ::glClear(GL_COLOR_BUFFER_BIT);
    ::glDisable(GL_DEPTH_TEST);
  };

  void setView2d() {
    ::glMatrixMode(GL_PROJECTION);
    ::glLoadIdentity();

    // Use glOrtho instead of gluOrtho2D
    ::glOrtho(0.0, w(), 0.0, h(), -1.0, 1.0);

    ::glMatrixMode(GL_MODELVIEW);
    ::glLoadIdentity();

    ::glTranslatef(transform_2d_.move_x, transform_2d_.move_y, 0.0f);
    ::glScalef(transform_2d_.scale, transform_2d_.scale, 1.0f);
  }

  void finish2d() { ::glFinish(); };

  //
  // 3D Functions
  //

  void initGL() { initGL(false, false); };
  void initGL(bool isTransparency, bool isLineSmooth) {

    // GLAD is initialized
    initGLAD();

    // Initialize light positions first
    initLightPositions();

    ::glPolygonMode(GL_BACK, GL_FILL);
    //::glCullFace( GL_BACK );
    //   ::glEnable( GL_CULL_FACE );

    // enable depth testing
    ::glEnable(GL_DEPTH_TEST);
    //   ::glDepthFunc( GL_LEQUAL );
    ::glDepthFunc(GL_LESS);
    //   ::glClearDepth( 1.0f );

    ::glEnable(GL_NORMALIZE);

    // transparency settings
    if (isTransparency) {
      ::glEnable(GL_ALPHA_TEST);
      ::glEnable(GL_BLEND);
      ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    ::glEnable(GL_POLYGON_OFFSET_FILL);
    ::glEnable(GL_POLYGON_OFFSET_LINE);
    ::glPolygonOffset((float)1.0, (float)1e-5);

    // line anti-aliasing
    if (isLineSmooth) {
      ::glEnable(GL_LINE_SMOOTH);
      ::glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    }

    const GLubyte* version = glGetString(GL_VERSION);
    const GLubyte* renderer = glGetString(GL_RENDERER);
    std::cout << "OpenGL Version: " << version << std::endl;
    std::cout << "OpenGL Renderer: " << renderer << std::endl;
  };

  void initLightPositions() {
    //
    // カメラ位置と注視点から自動的に三点照明を配置する
    //

    Eigen::Vector3f camera_pos = view_params_.view_point;
    Eigen::Vector3f look_at = view_params_.look_point;

    // 視線ベクトルとカメラの右方向・上方向を定義
    Eigen::Vector3f view_dir = (look_at - camera_pos).normalized();
    Eigen::Vector3f up_dir(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f right_dir = view_dir.cross(up_dir).normalized();
    Eigen::Vector3f true_up = right_dir.cross(view_dir).normalized();

    // 被写体中心を照らす基準位置（ターゲットより少し後ろ）
    Eigen::Vector3f base = look_at - 1.0f * view_dir;

    // Key Light: 右上前
    light_position_[0] << base + 2.0f * right_dir + 2.0f * true_up, 1.0f; // 点光源

    // Fill Light: 左上前
    light_position_[1] << base - 2.0f * right_dir + 1.0f * true_up, 1.0f; // 点光源

    // Rim Light: 真後ろ上
    light_position_[2] << look_at - 3.0f * view_dir + 2.0f * true_up, 1.0f; // 点光源

    // 平行光源（下から）
    light_position_[3] << 0.0f, -1.0f, 0.0f, 0.0f;
    
    // すべて有効にする
    for (int i = 0; i < 4; ++i) light_enabled_[i] = true;
  };

  void changeSize(int w, int h) {
    setW(w);
    setH(h);
    viewport_.aspect = static_cast<float>(w) / static_cast<float>(h);
    manip_.setHalfWHL((int)(static_cast<float>(w) / 2.0f),
                      (int)(static_cast<float>(h) / 2.0f));
  };

  //
  // draw functions
  //
  void clear() { clear(w(), h()); };
  void clear(int w, int h) {
    ::glViewport(0, 0, w, h);
    ::glClearColor(bgrgb_[0], bgrgb_[1], bgrgb_[2], 0.0f);
    ::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // draw gradient background
    if (isGradientBackground()) drawGradientBackground();
    //::glEnable(GL_DEPTH_TEST);
  };

  // Draw gradient background
  void drawGradientBackground() {
    ::glDisable(GL_DEPTH_TEST);
    ::glDisable(GL_CULL_FACE);
    ::glUseProgram(shader_.gradShaderProgram);
    ::glBindVertexArray(gradVAO_);
    ::glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    ::glBindVertexArray(0);
    ::glUseProgram(0);
    ::glEnable(GL_DEPTH_TEST);
    ::glEnable(GL_CULL_FACE);
  };

  ////////////////////////////////////////////////////////////////////////

  // Create ModelView Matrix（いらないかも）
  // auto mv = createModelViewMatrix(view_point, look_point);
  // glUniformMatrix4fv(modelviewLocation, 1, GL_FALSE, mv.data());
  Eigen::Matrix4f createModelViewMatrix(const Eigen::Vector3f& view_point,
                                        const Eigen::Vector3f& look_point) {
    Eigen::Vector3f forward = (look_point - view_point).normalized();
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f side = forward.cross(up).normalized();
    up = side.cross(forward);  // 正確な直交基底を再計算

    // 回転部分（列優先）
    Eigen::Matrix4f modelview = Eigen::Matrix4f::Identity();
    modelview(0, 0) = side.x();
    modelview(1, 0) = side.y();
    modelview(2, 0) = side.z();
    modelview(0, 1) = up.x();
    modelview(1, 1) = up.y();
    modelview(2, 1) = up.z();
    modelview(0, 2) = -forward.x();
    modelview(1, 2) = -forward.y();
    modelview(2, 2) = -forward.z();

    // 平行移動部分（dot積で直接構成）
    modelview(0, 3) = -side.dot(view_point);
    modelview(1, 3) = -up.dot(view_point);
    modelview(2, 3) =
        forward.dot(view_point);  // forward は -Z 向きなので符号に注意

    return modelview;
  };

  // Arcball 付 Create ModelView Matrix
  Eigen::Matrix4f createModelViewMatrixArcball(
      const Eigen::Vector3f& view_point, const Eigen::Vector3f& look_point,
      const Eigen::Matrix4f& rotation, const Eigen::Vector3f& offset,
      float seezo) {
    // Step 1: LookAt ベースのビュー行列
    Eigen::Vector3f forward = (look_point - view_point).normalized();
    Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f side = forward.cross(up).normalized();
    up = side.cross(forward);

    Eigen::Matrix4f base = Eigen::Matrix4f::Identity();
    base(0, 0) = side.x();
    base(1, 0) = side.y();
    base(2, 0) = side.z();
    base(0, 1) = up.x();
    base(1, 1) = up.y();
    base(2, 1) = up.z();
    base(0, 2) = -forward.x();
    base(1, 2) = -forward.y();
    base(2, 2) = -forward.z();
    base(0, 3) = -side.dot(view_point);
    base(1, 3) = -up.dot(view_point);
    base(2, 3) = forward.dot(view_point);

    // Step 2: Arcball のズーム（Z方向への平行移動）
    Eigen::Matrix4f transZ = Eigen::Matrix4f::Identity();
    transZ(2, 3) = seezo;

    // Step 3: Arcball の回転（4x4の回転行列：manip_.mNow()）
    // rotation: Eigen::Matrix4f（列優先）で仮定

    // Step 4: Arcball の回転中心（オフセット）逆移動
    Eigen::Matrix4f transOffset = Eigen::Matrix4f::Identity();
    transOffset(0, 3) = -offset.x();
    transOffset(1, 3) = -offset.y();
    transOffset(2, 3) = -offset.z();

    // Step 5: 最終モデルビュー行列（列優先なので右から順に変換）
    Eigen::Matrix4f modelview = base * transZ * rotation * transOffset;
    return modelview;
  };

  // Create Normal Matrix
  // Eigen::Matrix3f normalMatrix = computeNormalMatrix(modelview);
  // glUniformMatrix3fv(normalMatrixLoc, 1, GL_FALSE, normalMatrix.data());
  Eigen::Matrix3f computeNormalMatrix(const Eigen::Matrix4f& modelview) {
    // 上位3×3部分（回転＋スケーリング）を抽出
    Eigen::Matrix3f mv3x3 = modelview.block<3, 3>(0, 0);

    // 法線変換行列：逆行列の転置（列優先）
    return mv3x3.inverse().transpose();
  };

  // Create Projection Matrix
  // auto proj = createProjectionMatrix(fov, aspect, near, far);
  // glUniformMatrix4fv(location, 1, GL_FALSE, proj.data());
  Eigen::Matrix4f createProjectionMatrix(float fov, float aspect,
                                         float near_plane, float far_plane) {
    float f = 1.0f / std::tan(fov * 0.5f * static_cast<float>(M_PI) / 180.0f);

    Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();

    projection(0, 0) = f / aspect;
    projection(1, 1) = f;
    projection(2, 2) = (far_plane + near_plane) / (near_plane - far_plane);
    projection(2, 3) =
        (2.0f * far_plane * near_plane) / (near_plane - far_plane);
    projection(3, 2) = -1.0f;

    return projection;
  };

  void update(GLMaterial& mtl) {
    updateProjViewLight();
    updateMaterial(mtl);
  };

  // void updateProjViewLight(Eigen::Matrix4f& proj, Eigen::Matrix4f& mv) {
  void updateProjViewLight() {
    glUseProgram(shader_.phongShaderProgram);
    // projection
    // ↓これはマイフレームなくても良い
    auto proj =
        createProjectionMatrix(projection_.fov, viewport_.aspect,
                               projection_.near_plane, projection_.far_plane);
    // ↓これはマイフレームないとダメ
    glUniformMatrix4fv(shader_.projectionLoc, 1, GL_FALSE, proj.data());
    // model view
    auto mv = createModelViewMatrixArcball(
        view_params_.view_point, view_params_.look_point, manip_.mNow(),
        manip_.offset(), manip_.seezo());
    glUniformMatrix4fv(shader_.modelviewLoc, 1, GL_FALSE, mv.data());
    // normal Matrix
    auto nmat = computeNormalMatrix(mv);
    glUniformMatrix3fv(shader_.normalmatrixLoc, 1, GL_FALSE, nmat.data());

    // Lights
    // ライトの位置（ワールド -> ビュー空間へ変換する必要あり）
    for (int i = 0; i < NUM_LIGHTS; ++i) {
      Eigen::Vector4f light_world =
          light_position_[i];  // 入力ライト座標（world space）
      Eigen::Vector4f light_view4 = mv * light_world;  // view space に変換
      Eigen::Vector4f light_view;

      if (light_world.w() == 0.0f) {
        // 平行光源（w = 0.0）: 方向を正規化し w = 0.0 に
        Eigen::Vector3f dir = light_view4.head<3>().normalized();
        light_view << dir[0], dir[1], dir[2], 0.0f;
      } else {
        // 点光源（w ≠ 0.0）: 射影して w = 1.0 に
        Eigen::Vector3f pos = light_view4.hnormalized();
        light_view << pos[0], pos[1], pos[2], 1.0f;
      }

      // デバッグ出力（任意）
      // std::cout << "light_view[" << i << "] = " << light_view.transpose() <<
      // std::endl;

      // glUniform4fv に変更
      glUniform4fv(shader_.lightpositionLoc[i], 1, light_view.data());
      glUniform1i(shader_.lightenabledLoc[i], light_enabled_[i] ? 1 : 0);
    }

    // wireframe
    glUseProgram(shader_.wireframeShaderProgram);
    glUniformMatrix4fv(shader_.wireframeprojectionLoc, 1, GL_FALSE,
                       proj.data());
    glUniformMatrix4fv(shader_.wireframemodelviewLoc, 1, GL_FALSE, mv.data());

    // lines3d
    glUseProgram(shader_.lines3dShaderProgram);
    // 行列は Eigen を使っていると仮定（行優先なので GL_FALSE）
    // modelview/projection は Eigen::Matrix4f
    glUniformMatrix4fv(shader_.lines3dProjectionLoc, 1, GL_FALSE, proj.data());
    glUniformMatrix4fv(shader_.lines3dModelViewLoc, 1, GL_FALSE, mv.data());
    // viewport_size: 解像度 (width, height)
    float w = static_cast<float>(viewport_.width);
    float h = static_cast<float>(viewport_.height);
    glUniform2f(shader_.lines3dViewportSizeLoc, w, h);
    // line_width: スクリーン空間での太さ（例: 2.0 ピクセル）
    glUniform1f(shader_.lines3dLineWidthLoc, 1.0f);
    // aspect: アスペクト比（width / height）
    glUniform1f(shader_.lines3dAspectLoc, w/h);
    // line color
    glUniform3f(shader_.lines3dLineColorLoc, 0.2f, 0.8f, 0.2f);  // RGB
  };

  void updateMaterial(GLMaterial& mtl) {
    // マテリアルパラメータ
    // GLMaterial& mtl = glmeshl.material();
    glUseProgram(shader_.phongShaderProgram);
    glUniform3fv(shader_.ambientcolorLoc, 1, mtl.ambient3().data());
    glUniform3fv(shader_.diffusecolorLoc, 1, mtl.diffuse3().data());
    glUniform3fv(shader_.emissioncolorLoc, 1, mtl.emission3().data());
    glUniform3fv(shader_.specularcolorLoc, 1, mtl.specular3().data());
    glUniform1f(shader_.shininessLoc, mtl.shininess());
  };

  float& fov() { return projection_.fov; };
  const float& fov() const { return projection_.fov; };
  float& aspect() { return viewport_.aspect; };
  const float& aspect() const { return viewport_.aspect; };
  float& near_plane() { return projection_.near_plane; };
  const float& near_plane() const { return projection_.near_plane; };
  float& far_plane() { return projection_.far_plane; };
  const float& far_plane() const { return projection_.far_plane; };

  void finish() {
    ::glFlush();

    ::glFinish();
  }

  Eigen::Vector3f& view_point() { return view_params_.view_point; };
  void setViewPoint(Eigen::Vector3f& p) { view_params_.view_point = p; };
  void setViewPoint(float x, float y, float z) {
    view_params_.view_point << x, y, z;
  };
  Eigen::Vector3f& view_vector() { return view_params_.view_vector; };
  void setViewVector(Eigen::Vector3f& p) { view_params_.view_vector = p; };
  void setViewVector(float x, float y, float z) {
    view_params_.view_vector << x, y, z;
  };
  Eigen::Vector3f& look_point() { return view_params_.look_point; };
  void setLookPoint(Eigen::Vector3f& p) { view_params_.look_point = p; };
  void setLookPoint(float x, float y, float z) {
    view_params_.look_point << x, y, z;
  };

  void setViewParameters(float width, float height, float fov, float nearP,
                         float farP) {
    projection_.fov = fov;
    projection_.near_plane = nearP;
    projection_.far_plane = farP;
    viewport_.aspect = width / height;
  };
  void setNearFarPlanes(float nearP, float farP) {
    projection_.near_plane = nearP;
    projection_.far_plane = farP;
  };
  void setFOV(float fov) { projection_.fov = fov; };

  void setMagObject(float f) { manip_.setMagObject(f); };

  // lights

  Eigen::Vector3f light_position3(int i) const {
    return light_position_[i].head<3>();  // x, y, z
  };

  Eigen::Vector4f light_position4(int i) const {
    return light_position_[i];  // x, y, z, w
  };

  void getLightPos(int i, std::array<float, 4>& lpos) {
    lpos[0] = light_position_[i].x();
    lpos[1] = light_position_[i].y();
    lpos[2] = light_position_[i].z();
    lpos[3] = light_position_[i].w();
  };

  void getInitLightPos(int i, Eigen::Vector3f& p) {
    p.x() = light_position_[i].x();
    p.y() = light_position_[i].y();
    p.z() = light_position_[i].z();
  };

  void getLightVec(int i, std::array<float, 4>& lvec) {
    lvec[0] = -light_position_[i].x();
    lvec[1] = -light_position_[i].y();
    lvec[2] = -light_position_[i].z();
    lvec[3] = 1.0f;
  };

  // background color functions
  float* bgColor() { return &(bgrgb_[0]); };
  void setBackgroundColor(unsigned char r, unsigned char g, unsigned char b) {
    bgrgb_[0] = (float)r / 255.0;
    bgrgb_[1] = (float)g / 255.0;
    bgrgb_[2] = (float)b / 255.0;
  };
  void setBackgroundColor(float r, float g, float b) {
    bgrgb_[0] = r;
    bgrgb_[1] = g;
    bgrgb_[2] = b;
  };

  void setSize(int w, int h) {
    setW(w);
    setH(h);
  };
  void setW(int w) { viewport_.width = w; };
  void setH(int h) { viewport_.height = h; };
  int w() const { return viewport_.width; };
  int h() const { return viewport_.height; };

  void reshape(int w, int h) {
    setW(w);
    setH(h);
    viewport_.aspect = static_cast<float>(w) / static_cast<float>(h);
    manip_.setHalfWHL((int)(w / 2.0), (int)(h / 2.0));
  };

  // rotate, translation and zoom function

  void setScreenXY(int x, int y) {
    manip_.setScrnXY(x, y);
    transform_2d_.x0 = x;
    transform_2d_.y0 = y;
    transform_2d_.s0 = y;
  };

  void startRotate() {
    setIsRotate(true);
    manip_.vFrom(manip_.mouse_on_sphere(manip_.scrn_x(), manip_.scrn_y(),
                                        manip_.halfW(), manip_.halfH()));
  };

  void startMove() { setIsMove(true); };
  void startZoom() { setIsZoom(true); };

  void updateRotate(int x, int y) {
    manip_.vTo(manip_.mouse_on_sphere(x, y, manip_.halfW(), manip_.halfH()));
    manip_.updateRotate(x, y);
    manip_.setScrnXY(x, y);
  };

  void updateMove(int x, int y) {
    manip_.updateMove(x, y, manip_.scrn_x(), manip_.scrn_y());
    manip_.setScrnXY(x, y);
  };

  void updateWheelZoom(float x) { manip_.updateWheelZoom(x); }

  void updateZoom(int x, int y) {
    manip_.updateZoom(x, y, manip_.scrn_x(), manip_.scrn_y());
    manip_.setScrnXY(x, y);
  };

  void updateMove2d(int x, int y) {
    transform_2d_.move_x += (float)(x - transform_2d_.x0);
    transform_2d_.x0 = x;
    transform_2d_.move_y -= (float)(y - transform_2d_.y0);
    transform_2d_.y0 = y;
  };

  void updateZoom2d(int x, int y) {
    transform_2d_.scale -=
        ZOOM_SENSITIVITY * static_cast<float>(y - transform_2d_.s0);
    transform_2d_.s0 = y;
    if (transform_2d_.scale < MIN_SCALE_2D) transform_2d_.scale = MIN_SCALE_2D;
  };

  void finishRMZ() {
    setIsRotate(false);
    setIsZoom(false);
    setIsMove(false);
    manip_.qDown(manip_.qNow());
    manip_.mDown(manip_.mNow());
    transform_2d_.x0 = 0;
    transform_2d_.y0 = 0;
    transform_2d_.s0 = 0;
  };

  void resetView2d() {
    transform_2d_.move_x = .0f;
    transform_2d_.move_y = .0f;
    transform_2d_.scale = 1.0f;
    transform_2d_.x0 = 0;
    transform_2d_.y0 = 0;
    transform_2d_.s0 = 0;
  };

  //
  bool isRotate() const { return transform_flags_.rotate; };
  bool isMove() const { return transform_flags_.move; };
  bool isZoom() const { return transform_flags_.zoom; };
  void setIsRotate(bool f) { transform_flags_.rotate = f; };
  void setIsMove(bool f) { transform_flags_.move = f; };
  void setIsZoom(bool f) { transform_flags_.zoom = f; };

  //
  // Texture functions
  //
  void initTexture() {
    if (texture_system_.num_units) return;

    ::glGetIntegerv(GL_MAX_TEXTURE_UNITS, &texture_system_.num_units);
    std::cout << "maximum texture units: " << texture_system_.num_units
              << std::endl;
    ::glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texture_system_.max_tex_size);
    std::cout << texture_system_.max_tex_size << " x "
              << texture_system_.max_tex_size << " max texture size."
              << std::endl;

    texture_system_.tex_objects.resize(texture_system_.num_units);
    texture_system_.tex_enabled.resize(texture_system_.num_units);
    for (int i = 0; i < texture_system_.num_units; ++i) {
      texture_system_.tex_enabled[i] = GL_FALSE;
    }
    ::glGenTextures(texture_system_.num_units,
                    &(texture_system_.tex_objects[0]));

    texture_system_.current_tex_id = 0;
  };

  // int loadTexture( const char* const filename,
  //                  std::vector<unsigned char>& img,
  //                  int* format, int* w, int* h ) {
  //
  // "non-use PNGImage" version
  unsigned int loadTexture(const std::vector<unsigned char>& image, int w,
                           int h, int channel) {
    // Parameter validation
    if (w <= 0 || h <= 0) {
      throw GLPanelException("Invalid texture dimensions: " +
                             std::to_string(w) + "x" + std::to_string(h));
    }
    if (channel != 3 && channel != 4) {
      throw GLPanelException("Unsupported channel count: " +
                             std::to_string(channel));
    }
    if (image.size() != static_cast<size_t>(w * h * channel)) {
      throw GLPanelException("Image data size mismatch");
    }

    // the first load
    if (!texture_system_.num_units) initTexture();
    if (texture_system_.current_tex_id >= texture_system_.num_units) {
      throw GLPanelException("No available texture units");
    }

    int i = texture_system_.current_tex_id;
    ::glBindTexture(GL_TEXTURE_2D, texture_system_.tex_objects[i]);

    // Set texture parameters
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                      GL_NEAREST_MIPMAP_NEAREST);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // Load texture data
    if (channel == 3) {
      ::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, image.data());
    } else if (channel == 4) {
      ::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, image.data());
    }

    checkOpenGLError("Loading texture data");

    // Generate mipmaps
    ::glGenerateMipmap(GL_TEXTURE_2D);
    checkOpenGLError("Generating mipmaps");

    texture_system_.tex_enabled[i] = GL_TRUE;
    texture_system_.current_tex_id++;

    ::glBindTexture(GL_TEXTURE_2D, 0);

    return texture_system_.tex_objects[i];
  };

  void assignTexture(int id, std::vector<unsigned char>& img, int format, int w,
                     int h) {
    ::glBindTexture(GL_TEXTURE_2D, id);

    ::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                      GL_NEAREST_MIPMAP_NEAREST);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    ::glTexImage2D(GL_TEXTURE_2D, 0, format, w, h, 0, format, GL_UNSIGNED_BYTE,
                   img.data());

    ::glGenerateMipmap(GL_TEXTURE_2D);

    ::glBindTexture(GL_TEXTURE_2D, 0);
  };

  //
  // window capture functions
  //
  void capture(std::vector<unsigned char>& image, int w, int h, int channel) {
    // Ensure image vector has enough space
    image.resize(w * h * channel);

    // temporal capture
    std::vector<unsigned char> tcap(w * h * channel);
    ::glFlush();
    if (channel == 4)  // RGB + depth
      ::glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, &(tcap[0]));
    else if (channel == 3)  // RGB
      ::glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, &(tcap[0]));

    // flip is required
    for (int i = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j) {
        int ni = channel * (w * (h - 1 - i) + j);
        int pi = channel * (w * i + j);
        for (int k = 0; k < channel; ++k) {
          image[ni + k] = tcap[pi + k];
        }
      }
    }
  };

  //
  // flag functions
  //

  // gradient background functions
  void setIsGradientBackground(bool f) {
    display_flags_.gradient_background = f;
  };
  bool isGradientBackground() const {
    return display_flags_.gradient_background;
  };

  // arcball
  Arcball& manip() { return manip_; };

 private:
  // Viewport information
  struct Viewport {
    int width = 0;
    int height = 0;
    float aspect = 1.0f;
  } viewport_;

  // Projection parameters
  struct Projection {
    float fov = DEFAULT_FOV;
    float near_plane = DEFAULT_NEAR_PLANE;
    float far_plane = DEFAULT_FAR_PLANE;
  } projection_;

  // shader parameters
  GLShader shader_;

  GLuint gradVAO_;
  GLuint gradVBO_;

  // Background color
  std::array<float, BACKGROUND_COLOR_SIZE> bgrgb_;

  // Light system
  std::array<Eigen::Vector4f, NUM_LIGHTS>
      light_position_;                          // 4 lights * 4 components
  std::array<bool, NUM_LIGHTS> light_enabled_;  // 有効フラグ

  // Transformation flags
  struct TransformFlags {
    bool rotate = false;
    bool move = false;
    bool zoom = false;
  } transform_flags_;

  // Texture system
  struct TextureSystem {
    int num_units = 0;
    std::vector<unsigned int> tex_objects;
    std::vector<bool> tex_enabled;
    int current_tex_id = 0;
    int max_tex_size = 0;
  } texture_system_;

  // Display flags
  struct DisplayFlags {
    bool draw_wireframe = false;
    bool draw_shading = false;
    bool gradient_background = true;
  } display_flags_;

  // Arcball manipulator
  Arcball manip_;

  // 2D transformation state
  struct Transform2D {
    float move_x = 0.0f;
    float move_y = 0.0f;
    float scale = 1.0f;
    int x0 = 0;
    int y0 = 0;
    int s0 = 0;
  } transform_2d_;

  // View parameters
  struct ViewParams {
    Eigen::Vector3f view_point;
    Eigen::Vector3f view_vector;
    Eigen::Vector3f look_point;
  } view_params_;

  // OpenGL error checking helper
  void checkOpenGLError(const std::string& operation) {
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
      std::string error_msg = "OpenGL error in " + operation + ": ";
      switch (error) {
        case GL_INVALID_ENUM:
          error_msg += "GL_INVALID_ENUM";
          break;
        case GL_INVALID_VALUE:
          error_msg += "GL_INVALID_VALUE";
          break;
        case GL_INVALID_OPERATION:
          error_msg += "GL_INVALID_OPERATION";
          break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
          error_msg += "GL_INVALID_FRAMEBUFFER_OPERATION";
          break;
        case GL_OUT_OF_MEMORY:
          error_msg += "GL_OUT_OF_MEMORY";
          break;
        default:
          error_msg += "Unknown error " + std::to_string(error);
          break;
      }
      throw OpenGLException(error_msg);
    }
  };
};

#endif  // _GLPANEL_HXX
