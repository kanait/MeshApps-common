////////////////////////////////////////////////////////////////////
//
// $Id: GLMeshL.hxx 2025/07/19 15:23:19 kanai Exp 
//
// OpenGL MeshL VBO/VAO draw class
//
// Copyright (c) 2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _GLMESHL_HXX
#define _GLMESHL_HXX 1

#include "myGL.hxx"

#include "MeshL.hxx"
#include <set>
#include <iostream>
#include "GLMesh.hxx"

#include "GLShader.hxx"
#include "GLMaterial.hxx"

struct VertexAttribBasic {
  Eigen::Vector3f position;
  Eigen::Vector3f normal;
};

struct VertexAttribColor {
  Eigen::Vector3f position;
  Eigen::Vector3f normal;
  Eigen::Vector3f color;
};

class GLMeshL : public GLMesh {
 public:
  GLMeshL() : meshl_(nullptr) {};
  ~GLMeshL() {
    // deleteVAOVBO();
  };

  void deleteVAOVBO() {
    if (vao_flat_ != 0) {
      glDeleteVertexArrays(1, &vao_flat_);
      vao_flat_ = 0;
    }
    if (vbo_flat_ != 0) {
      glDeleteBuffers(1, &vbo_flat_);
      vbo_flat_ = 0;
    }
    if (vao_smooth_ != 0) {
      glDeleteVertexArrays(1, &vao_smooth_);
      vao_smooth_ = 0;
    }
    if (vbo_smooth_ != 0) {
      glDeleteBuffers(1, &vbo_smooth_);
      vbo_smooth_ = 0;
    }
    if (vao_wire_ != 0) {
      glDeleteVertexArrays(1, &vao_wire_);
      vao_wire_ = 0;
    }
    if (vbo_wire_ != 0) {
      glDeleteBuffers(1, &vbo_wire_);
      vbo_wire_ = 0;
    }
  };

  void setMesh(std::shared_ptr<MeshL> mesh) {
    meshl_ = mesh;
    meshl_->calcSmoothVertexNormal();
    buildBuffers();
  };

  const std::shared_ptr<MeshL> mesh() const { return meshl_; };
  bool empty() const { return (meshl_ != NULL) ? false : true; };

  void draw(GLShader& shader) {
    if ((isSmoothShading() == false) && (isDrawWireframe() == false)) {
      // flat shading
      // std::cout << "program " << shader.phongShaderProgram << std::endl;
      glUseProgram(shader.phongShaderProgram);
      glBindVertexArray(vao_flat_);
      glDrawArrays(GL_TRIANGLES, 0, vertex_count_flat_);
      glBindVertexArray(0);
    } else if ((isSmoothShading() == true) && (isDrawWireframe() == false)) {
      // smooth shading
      glUseProgram(shader.phongShaderProgram);
      glBindVertexArray(vao_smooth_);
      glDrawArrays(GL_TRIANGLES, 0, vertex_count_smooth_);
      glBindVertexArray(0);
    } else if ((isSmoothShading() == false) && (isDrawWireframe() == true)) {
      // ワイヤフレーム＋フラットシェーディング
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_STENCIL_TEST);

      // ===== パス 1: 塗りつぶし (flat shading) =====
      glUseProgram(shader.phongShaderProgram);
      glBindVertexArray(vao_flat_);  // flat shading 用の VAO に変更

      // 深度テスト有効・深度書き込みも有効
      glDepthMask(GL_TRUE);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

      // ステンシルバッファに前面のピクセルをマーク
      glStencilFunc(GL_ALWAYS, 1, 0xFF);
      glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
      glStencilMask(0xFF);

      // 通常の描画
      glDrawArrays(GL_TRIANGLES, 0,
                   vertex_count_flat_);  // flat buffer の頂点数

      // ===== パス 2: ワイヤフレーム描画 =====
      // 2パス方式：シンプルで高速

      // パス 1: ソリッドメッシュ描画
      glUseProgram(shader.phongShaderProgram);
      glBindVertexArray(vao_flat_);

      // 後面カリング有効（背面のワイヤーフレームを減らす）
      glEnable(GL_CULL_FACE);
      glCullFace(GL_BACK);

      // 深度テスト有効
      glDepthMask(GL_TRUE);
      glDepthFunc(GL_LESS);

      // ソリッドメッシュ描画
      glDrawArrays(GL_TRIANGLES, 0, vertex_count_flat_);

      // パス 2: ワイヤーフレーム描画
      glUseProgram(shader.lines3dShaderProgram);
      glUniform1f(shader.lines3dLineWidthLoc,
                  1.2f);  // 線の太さを少し増やしてアンチエイリアシング効果向上
      glUniform3f(shader.lines3dLineColorLoc, 0.0f, 0.0f, 0.0f);  // RGB - 黒色
      glUniform1f(shader.lines3dDepthOffsetLoc,
                  0.00002f);  // シェーダーベースのポリゴンオフセット

      glBindVertexArray(vao_wire_);

      // 深度テスト有効、深度書き込みは無効
      glDepthMask(GL_FALSE);
      glDepthFunc(GL_LEQUAL);

      // アンチエイリアシング
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      // マルチサンプリング有効化（利用可能な場合）
      glEnable(GL_MULTISAMPLE);

      // 線の滑らかさを向上
      glEnable(GL_LINE_SMOOTH);
      glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

      // ワイヤーフレーム描画
      glDrawArrays(GL_LINES, 0, vertex_count_wire_);

      // 後始末
      glDisable(GL_BLEND);
      glDisable(GL_CULL_FACE);
      glDisable(GL_LINE_SMOOTH);
      glDisable(GL_MULTISAMPLE);
      glBindVertexArray(0);
      glDepthMask(GL_TRUE);
      glDepthFunc(GL_LESS);
    }
  };

  GLuint vao_flat() const { return vao_flat_; };
  GLuint vertex_count_flat() const { return vertex_count_flat_; };
  GLuint vao_smooth() const { return vao_smooth_; };
  GLuint vertex_count_smooth() const { return vertex_count_smooth_; };
  GLuint vao_wire() const { return vao_wire_; };
  GLuint vertex_count_wire() const { return vertex_count_wire_; };

  void buildBuffers() {
    // === Flat Shading ===
    auto flat_buffer =
        generateFlatShadingVertexBuffer<VertexAttribBasic>(*meshl_);
    vertex_count_flat_ = flat_buffer.size();

    setupVAO<VertexAttribBasic>(vao_flat_, vbo_flat_, flat_buffer,
                                {{0, offsetof(VertexAttribBasic, position)},
                                 {1, offsetof(VertexAttribBasic, normal)}});

    // === Smooth Shading ===
    auto smooth_buffer =
        generateSmoothShadingVertexBuffer<VertexAttribColor>(*meshl_);
    vertex_count_smooth_ = smooth_buffer.size();

    setupVAO<VertexAttribColor>(vao_smooth_, vbo_smooth_, smooth_buffer,
                                {{0, offsetof(VertexAttribColor, position)},
                                 {1, offsetof(VertexAttribColor, normal)},
                                 {2, offsetof(VertexAttribColor, color)}});

    // === Wireframe (flat) ===
    auto wire_buffer = generateWireframeVertexBuffer(*meshl_);
    initLines3dVAO(wire_buffer);
  };

  void initLines3dVAO(std::vector<float>& lines_buffer) {
    glGenVertexArrays(1, &vao_wire_);
    glBindVertexArray(vao_wire_);

    GLuint vbo = 0;
    glGenBuffers(1, &vbo_wire_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_wire_);
    glBufferData(GL_ARRAY_BUFFER, lines_buffer.size() * sizeof(float),
                 lines_buffer.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,  // location = 0
                          3,  // 3 floats per vertex
                          GL_FLOAT, GL_FALSE,
                          3 * sizeof(float),  // stride: 1 vertex = 3 floats
                          (void*)0);          // offset

    glBindVertexArray(0);
  };

  GLMaterial& material() { return mtln_; };
  const GLMaterial& material() const { return mtln_; };

 private:
  std::shared_ptr<MeshL> meshl_;

  GLuint vao_flat_ = 0, vbo_flat_ = 0;
  GLuint vao_smooth_ = 0, vbo_smooth_ = 0;
  GLuint vao_wire_ = 0, vbo_wire_ = 0;
  GLuint vertex_count_flat_ = 0;
  GLuint vertex_count_smooth_ = 0;
  GLuint vertex_count_wire_ = 0;

  GLMaterial mtln_;

  std::vector<float> generateWireframeVertexBuffer(MeshL& mesh) {
    std::vector<float> buffer;
    int num_lines = 0;

    // すべての辺を描画（重複除去）
    std::set<std::pair<int, int>> processed_edges;

    for (const auto& face : mesh.faces()) {
      for (auto& he : face->halfedges()) {
        // Get vertex IDs for this edge
        int v1_id = he->vertex()->id();
        int v2_id = he->next()->vertex()->id();

        // Create a consistent edge representation (smaller ID first)
        std::pair<int, int> edge = (v1_id < v2_id)
                                       ? std::make_pair(v1_id, v2_id)
                                       : std::make_pair(v2_id, v1_id);

        // Only process each edge once
        if (processed_edges.find(edge) == processed_edges.end()) {
          processed_edges.insert(edge);

          // すべての辺を描画
          Eigen::Vector3f p1 = he->vertex()->point().cast<float>();
          buffer.push_back(p1.x());
          buffer.push_back(p1.y());
          buffer.push_back(p1.z());
          Eigen::Vector3f p2 = he->next()->vertex()->point().cast<float>();
          buffer.push_back(p2.x());
          buffer.push_back(p2.y());
          buffer.push_back(p2.z());
          num_lines++;
        }
      }
    }
    vertex_count_wire_ = num_lines * 2;
    // std::cout << "Generated wireframe buffer: " << num_lines << " lines, " <<
    // vertex_count_wire_ << " vertices" << std::endl;
    return buffer;
  };

  inline std::vector<VertexAttribColor>
  generateFlatShadingVertexBufferFixedColor(
      MeshL& mesh, const Eigen::Vector3f& fixedColor =
                       Eigen::Vector3f(0.0f, 0.0f, 0.0f))  // デフォルトは白
  {
    std::vector<VertexAttribColor> buffer;

    for (const auto& face : mesh.faces()) {
      std::vector<std::shared_ptr<HalfedgeL>> he_list;
      for (auto& he : face->halfedges()) {
        he_list.push_back(he);
      }

      if (he_list.size() >= 3) {
        for (size_t i = 1; i + 1 < he_list.size(); ++i) {
          auto h0 = he_list[0], h1 = he_list[i], h2 = he_list[i + 1];

          Eigen::Vector3f p0 = h0->vertex()->point().cast<float>();
          Eigen::Vector3f p1 = h1->vertex()->point().cast<float>();
          Eigen::Vector3f p2 = h2->vertex()->point().cast<float>();

          Eigen::Vector3f n = (p1 - p0).cross(p2 - p0).normalized();

          buffer.push_back({p0, n, fixedColor});
          buffer.push_back({p1, n, fixedColor});
          buffer.push_back({p2, n, fixedColor});
        }
      }
    }

    return buffer;
  };

  // VAO Setup
  template <typename VertexAttribT>
  void setupVAO(GLuint& vao, GLuint& vbo,
                const std::vector<VertexAttribT>& buffer,
                const std::vector<std::pair<GLuint, size_t>>& attribLayout) {
    // 既存の VAO/VBO を削除（0 でない場合のみ）
    if (vao != 0) {
      glDeleteVertexArrays(1, &vao);
      vao = 0;
    }
    if (vbo != 0) {
      glDeleteBuffers(1, &vbo);
      vbo = 0;
    }

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexAttribT) * buffer.size(),
                 buffer.data(), GL_STATIC_DRAW);

    for (const auto& [index, offset] : attribLayout) {
      glVertexAttribPointer(index, 3, GL_FLOAT, GL_FALSE, sizeof(VertexAttribT),
                            (void*)offset);
      glEnableVertexAttribArray(index);
    }

    glBindVertexArray(0);
  };

  // テンプレート関数の実装をクラス内に定義
  template <typename VertexAttribT>
  inline std::vector<VertexAttribT> generateFlatShadingVertexBuffer(
      MeshL& mesh) {
    std::vector<VertexAttribT> buffer;
    for (const auto& face : mesh.faces()) {
      std::vector<std::shared_ptr<HalfedgeL>> he_list(face->halfedges().begin(),
                                                      face->halfedges().end());
      if (he_list.size() >= 3) {
        for (size_t i = 1; i + 1 < he_list.size(); ++i) {
          auto h0 = he_list[0], h1 = he_list[i], h2 = he_list[i + 1];
          Eigen::Vector3f p0 = h0->vertex()->point().cast<float>();
          Eigen::Vector3f p1 = h1->vertex()->point().cast<float>();
          Eigen::Vector3f p2 = h2->vertex()->point().cast<float>();
          // Eigen::Vector3f n = (p1 - p0).cross(p2 - p0).normalized();
          Eigen::Vector3f n = face->normal().cast<float>();

          if constexpr (std::is_same_v<VertexAttribT, VertexAttribBasic>) {
            buffer.push_back({p0, n});
            buffer.push_back({p1, n});
            buffer.push_back({p2, n});
          } else if constexpr (std::is_same_v<VertexAttribT,
                                              VertexAttribColor>) {
            Eigen::Vector3f color(0.0f, 0.0f, 0.0f);  // 固定色
            buffer.push_back({p0, n, color});
            buffer.push_back({p1, n, color});
            buffer.push_back({p2, n, color});
          }
        }
      }
    }
    return buffer;
  }

  template <typename VertexAttribT>
  inline std::vector<VertexAttribT> generateSmoothShadingVertexBuffer(
      MeshL& mesh) {
    std::vector<VertexAttribT> buffer;
    for (const auto& face : mesh.faces()) {
      std::vector<std::shared_ptr<HalfedgeL>> he_list(face->halfedges().begin(),
                                                      face->halfedges().end());
      if (he_list.size() >= 3) {
        for (size_t i = 1; i + 1 < he_list.size(); ++i) {
          auto h0 = he_list[0], h1 = he_list[i], h2 = he_list[i + 1];

          if constexpr (std::is_same_v<VertexAttribT, VertexAttribBasic>) {
            buffer.push_back(
                {h0->vertex()->point().cast<float>(),
                 h0->normal()
                     ? Eigen::Vector3f(h0->normal()->point().cast<float>())
                     : Eigen::Vector3f::Zero()});
            buffer.push_back(
                {h1->vertex()->point().cast<float>(),
                 h1->normal()
                     ? Eigen::Vector3f(h1->normal()->point().cast<float>())
                     : Eigen::Vector3f::Zero()});
            buffer.push_back(
                {h2->vertex()->point().cast<float>(),
                 h2->normal()
                     ? Eigen::Vector3f(h2->normal()->point().cast<float>())
                     : Eigen::Vector3f::Zero()});
          } else if constexpr (std::is_same_v<VertexAttribT,
                                              VertexAttribColor>) {
            Eigen::Vector3f color(0.0f, 0.0f, 0.0f);  // 固定色
            buffer.push_back(
                {h0->vertex()->point().cast<float>(),
                 h0->normal()
                     ? Eigen::Vector3f(h0->normal()->point().cast<float>())
                     : Eigen::Vector3f::Zero(),
                 color});
            buffer.push_back(
                {h1->vertex()->point().cast<float>(),
                 h1->normal()
                     ? Eigen::Vector3f(h1->normal()->point().cast<float>())
                     : Eigen::Vector3f::Zero(),
                 color});
            buffer.push_back(
                {h2->vertex()->point().cast<float>(),
                 h2->normal()
                     ? Eigen::Vector3f(h2->normal()->point().cast<float>())
                     : Eigen::Vector3f::Zero(),
                 color});
          }
        }
      }
    }
    return buffer;
  };

};  // class GLMeshL

#endif  // _GLMESHL_HXX
