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

struct VertexAttribTextured {
  Eigen::Vector3f position;
  Eigen::Vector3f normal;
  Eigen::Vector2f texcoord;
};

struct AttribDesc {
  GLuint index;
  GLint size;
  size_t offset;
};

class GLMeshL : public GLMesh {
 public:
  GLMeshL() : meshl_(nullptr), tex_id_(0), has_textured_buffer_(false) {};
  // Globals are destroyed after glfwTerminate(); do not call OpenGL here.
  ~GLMeshL() { resetVAOVBOHandles(); };

  // Requires a current OpenGL context. Call before glfwDestroyWindow/glfwTerminate.
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
    if (vao_texture_ != 0) {
      glDeleteVertexArrays(1, &vao_texture_);
      vao_texture_ = 0;
    }
    if (vbo_texture_ != 0) {
      glDeleteBuffers(1, &vbo_texture_);
      vbo_texture_ = 0;
    }
    has_textured_buffer_ = false;
    vertex_count_texture_ = 0;
  };

  void setMesh(std::shared_ptr<MeshL> mesh) {
    meshl_ = mesh;
    meshl_->calcSmoothVertexNormal();
    buildBuffers();
  };

  const std::shared_ptr<MeshL> mesh() const { return meshl_; };
  bool empty() const { return (meshl_ != NULL) ? false : true; };

  void setTexID(unsigned int id) { tex_id_ = id; };
  unsigned int texID() const { return tex_id_; };
  bool hasTexturedBuffer() const { return has_textured_buffer_; };

  void drawTextured(GLShader& shader) {
    if (!has_textured_buffer_ || tex_id_ == 0) return;

    glUseProgram(shader.textureShaderProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex_id_);
    glBindVertexArray(vao_texture_);
    glDrawArrays(GL_TRIANGLES, 0, vertex_count_texture_);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
  };

  void drawSolid(GLShader& shader) {
    if (isDrawTexture() && has_textured_buffer_) return;

    glUseProgram(shader.phongShaderProgram);
    if (isSmoothShading()) {
      glBindVertexArray(vao_smooth_);
      glDrawArrays(GL_TRIANGLES, 0, vertex_count_smooth_);
    } else {
      glBindVertexArray(vao_flat_);
      glDrawArrays(GL_TRIANGLES, 0, vertex_count_flat_);
    }
    glBindVertexArray(0);
  };

  void drawWireOverlay(GLShader& shader) {
    if (vao_wire_ == 0 || vertex_count_wire_ == 0) return;

    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glUseProgram(shader.lines3dShaderProgram);
    glUniform1f(shader.lines3dLineWidthLoc, 1.2f);
    glUniform3f(shader.lines3dLineColorLoc, 0.0f, 0.0f, 0.0f);
    glUniform1f(shader.lines3dDepthOffsetLoc, 0.00002f);

    glBindVertexArray(vao_wire_);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glDrawArrays(GL_LINES, 0, vertex_count_wire_);
    glDisable(GL_BLEND);
    glDisable(GL_LINE_SMOOTH);
    glBindVertexArray(0);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
  };

  void draw(GLShader& shader) {
    drawSolid(shader);
    if (isDrawWireframe()) drawWireOverlay(shader);
  };


  GLuint vao_flat() const { return vao_flat_; };
  GLuint vertex_count_flat() const { return vertex_count_flat_; };
  GLuint vao_smooth() const { return vao_smooth_; };
  GLuint vertex_count_smooth() const { return vertex_count_smooth_; };
  GLuint vao_wire() const { return vao_wire_; };
  GLuint vertex_count_wire() const { return vertex_count_wire_; };

  // Update GPU buffers after deforming mesh topology/vertex positions in place.
  void updateBuffersFromMesh() {
    if (!meshl_) return;
    if (vao_smooth_ == 0) {
      buildBuffers();
      return;
    }

    meshl_->calcSmoothVertexNormal();

    auto smooth_buffer =
        generateSmoothShadingVertexBuffer<VertexAttribColor>(*meshl_);
    vertex_count_smooth_ = smooth_buffer.size();
    glBindBuffer(GL_ARRAY_BUFFER, vbo_smooth_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexAttribColor) * smooth_buffer.size(),
                 smooth_buffer.data(), GL_DYNAMIC_DRAW);

    auto flat_buffer =
        generateFlatShadingVertexBuffer<VertexAttribBasic>(*meshl_);
    vertex_count_flat_ = flat_buffer.size();
    glBindBuffer(GL_ARRAY_BUFFER, vbo_flat_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexAttribBasic) * flat_buffer.size(),
                 flat_buffer.data(), GL_DYNAMIC_DRAW);

    if (isDrawWireframe() && vao_wire_ != 0) {
      auto wire_buffer = generateWireframeVertexBuffer(*meshl_);
      glBindBuffer(GL_ARRAY_BUFFER, vbo_wire_);
      glBufferData(GL_ARRAY_BUFFER, wire_buffer.size() * sizeof(float),
                   wire_buffer.data(), GL_DYNAMIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
  };

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

    // === Textured flat shading ===
    if (!meshl_->texcoords().empty()) {
      auto texture_buffer = generateTexturedFlatVertexBuffer(*meshl_);
      vertex_count_texture_ = texture_buffer.size();
      has_textured_buffer_ = (vertex_count_texture_ > 0);
      if (has_textured_buffer_) {
        setupVAOEx(vao_texture_, vbo_texture_, texture_buffer,
                   {{0, 3, offsetof(VertexAttribTextured, position)},
                    {1, 3, offsetof(VertexAttribTextured, normal)},
                    {2, 2, offsetof(VertexAttribTextured, texcoord)}});
      }
    } else {
      has_textured_buffer_ = false;
      vertex_count_texture_ = 0;
    }
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
  void resetVAOVBOHandles() {
    vao_flat_ = vbo_flat_ = 0;
    vao_smooth_ = vbo_smooth_ = 0;
    vao_wire_ = vbo_wire_ = 0;
    vao_texture_ = vbo_texture_ = 0;
    has_textured_buffer_ = false;
    vertex_count_texture_ = 0;
  }

  std::shared_ptr<MeshL> meshl_;

  GLuint vao_flat_ = 0, vbo_flat_ = 0;
  GLuint vao_smooth_ = 0, vbo_smooth_ = 0;
  GLuint vao_wire_ = 0, vbo_wire_ = 0;
  GLuint vao_texture_ = 0, vbo_texture_ = 0;
  GLuint vertex_count_flat_ = 0;
  GLuint vertex_count_smooth_ = 0;
  GLuint vertex_count_wire_ = 0;
  GLuint vertex_count_texture_ = 0;

  unsigned int tex_id_;
  bool has_textured_buffer_;

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

  template <typename VertexAttribT>
  void setupVAOEx(GLuint& vao, GLuint& vbo,
                  const std::vector<VertexAttribT>& buffer,
                  const std::vector<AttribDesc>& attribLayout) {
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

    for (const auto& desc : attribLayout) {
      glVertexAttribPointer(desc.index, desc.size, GL_FLOAT, GL_FALSE,
                            sizeof(VertexAttribT), (void*)desc.offset);
      glEnableVertexAttribArray(desc.index);
    }

    glBindVertexArray(0);
  };

  std::vector<VertexAttribTextured> generateTexturedFlatVertexBuffer(
      MeshL& mesh) {
    std::vector<VertexAttribTextured> buffer;

    for (const auto& face : mesh.faces()) {
      std::vector<std::shared_ptr<HalfedgeL>> he_list(face->halfedges().begin(),
                                                      face->halfedges().end());
      if (he_list.size() < 3) continue;

      bool has_uv = true;
      for (const auto& he : he_list) {
        if (!he->isTexcoord()) {
          has_uv = false;
          break;
        }
      }
      if (!has_uv) continue;

      Eigen::Vector3f n = face->normal().cast<float>();

      for (size_t i = 1; i + 1 < he_list.size(); ++i) {
        auto h0 = he_list[0], h1 = he_list[i], h2 = he_list[i + 1];
        auto uv0 = [&](const std::shared_ptr<HalfedgeL>& he) {
          const Eigen::Vector3d& t = he->texcoord()->point();
          return Eigen::Vector2f(static_cast<float>(t.x()),
                                 static_cast<float>(t.y()));
        };

        buffer.push_back({h0->vertex()->point().cast<float>(), n, uv0(h0)});
        buffer.push_back({h1->vertex()->point().cast<float>(), n, uv0(h1)});
        buffer.push_back({h2->vertex()->point().cast<float>(), n, uv0(h2)});
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
