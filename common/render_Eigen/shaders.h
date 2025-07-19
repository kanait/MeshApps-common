////////////////////////////////////////////////////////////////////
//
// $Id: shaders.h 2025/07/19 15:38:44 kanai Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _SHADERS_H
#define _SHADERS_H 1

//
// OpenGL 3.3用
//

//
// shaders for gradient background
//

static const GLchar *gradVertShaderSrc = R"glsl(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec3 aColor;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)glsl";

static const GLchar *gradFragShaderSrc = R"glsl(
#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(vColor, 1.0);
}
)glsl";

//
// shaders for Phong shading
//

static const GLchar *vertex_shader_Phong_source33 = R"glsl(
#version 330 core

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;

uniform mat4 modelview;
uniform mat4 projection;
uniform mat3 normalMatrix;

out vec4 frag_position;
out vec3 frag_normal;

void main()
{
    frag_position = modelview * vec4(in_position, 1.0);
    frag_normal = normalize(normalMatrix * in_normal);
    gl_Position = projection * frag_position;
}
)glsl";

// ライト4つバージョン（点光源・平行光源 + 定数重み付き）
static const GLchar *fragment_shader_Phong_source33 = R"glsl(
#version 330 core

in vec4 frag_position;
in vec3 frag_normal;

uniform vec4 light_position[4];     // wで点光源/平行光源の区別
uniform bool light_enabled[4];

uniform vec3 ambient_color;
uniform vec3 diffuse_color;
uniform vec3 specular_color;
uniform vec3 emission_color;
uniform float shininess;

out vec4 fragColor;

void main()
{
    vec3 normal = normalize(frag_normal);
    vec3 viewDir = normalize(-frag_position.xyz);
    vec3 finalColor = vec3(0.0);

    // 各ライトの強さ係数（Key, Fill, Rim, Ambient風）
    //float light_weights[4] = float[4](0.7, 0.3, 0.5, 0.2);
    float light_weights[4] = float[4](0.56, 0.24, 0.40, 0.16); // 元の80%

    for (int i = 0; i < 4; ++i) {
      if (!light_enabled[i]) continue;

      vec3 lightDir;
      if (light_position[i].w == 0.0) {
        // 平行光源（方向）
        lightDir = normalize(-light_position[i].xyz);
      } else {
        // 点光源（位置）
        lightDir = normalize(light_position[i].xyz - frag_position.xyz);
      }

      vec3 halfwayDir = normalize(lightDir + viewDir);

      // シャドウ風の効果
      float ndotl = dot(normal, lightDir);
      float shadow = ndotl > 0.0 ? 1.0 : 0.1; // ← ここが擬似シャドウ

      float diff = max(ndotl, 0.0);
      float spec = pow(max(dot(normal, halfwayDir), 0.0), shininess);

      // 簡易AO
      //float ao = pow(1.0 - abs(dot(normal, vec3(0.0, 1.0, 0.0))), 2.0);  // 上からの遮蔽度
      //vec3 ambient = ambient_color * mix(0.3, 1.0, ao);
      vec3 ambient  = ambient_color;
      vec3 diffuse  = diffuse_color * diff * shadow;
      vec3 specular = specular_color * spec * shadow;

      // 重みを掛けて加算
      finalColor += (ambient + diffuse + specular) * light_weights[i];
    }
    finalColor += emission_color;

    fragColor = vec4(finalColor, 1.0);
}
)glsl";

//
// shaders for wireframe rendering
//

static const GLchar *vertex_wireframe_source33 = R"glsl(
#version 330 core

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_color;

out vec3 frag_color;

uniform mat4 modelview;
uniform mat4 projection;

void main()
{
    frag_color = in_color;
    gl_Position = projection * modelview * vec4(in_position, 1.0);
}
)glsl";

static const GLchar *fragment_wireframe_source33 = R"glsl(
  #version 330 core
  
  in vec3 frag_color;
  out vec4 fragColor;
  
  void main()
  {
    fragColor = vec4(frag_color, 1.0);
  }
)glsl";

//
// shaders for lines 3d rendering
//

static const GLchar *vertex_lines3d_source33 = R"glsl(
  #version 330 core
  layout(location = 0) in vec3 position;

  void main() {
    gl_Position = vec4(position, 1.0);
  }
)glsl";

static const GLchar *geometry_lines3d_source33 = R"glsl(
  #version 330 core
  layout(lines) in;
  layout(triangle_strip, max_vertices = 4) out;

  uniform mat4 modelview;
  uniform mat4 projection;
  uniform vec2 viewport_size; // 画面解像度（ピクセル）
  uniform float line_width;   // 線の太さ（ピクセル単位）
  uniform float aspect;       // 画面アスペクト比（width / height）
  
  out float v_dist;  // 線の中心からの距離（アンチエイリアシング用）

  void main()
  {
    // 射影変換
    vec4 p0 = projection * modelview * gl_in[0].gl_Position;
    vec4 p1 = projection * modelview * gl_in[1].gl_Position;

    // NDC 変換（クリップ空間 → -1~1）
    vec2 ndc0 = p0.xy / p0.w;
    vec2 ndc1 = p1.xy / p1.w;

    // 線の方向とその法線ベクトル
    vec2 dir = normalize(ndc1 - ndc0);
    vec2 normal = vec2(-dir.y, dir.x);

    // アスペクト比補正
    normal.x *= aspect;

    // ピクセル → NDC 換算：viewport の幅に対する比率
    float pixel_size = 2.0 / viewport_size.x;
    vec2 offset = normal * line_width * 0.5 * pixel_size;

    // 四角形を構成する 4 頂点出力（正しい深度値を保持）
    // 線の中心からの距離を計算してアンチエイリアシング用に出力
    v_dist = line_width * 0.5; gl_Position = vec4(ndc0 + offset, p0.z / p0.w, 1.0); EmitVertex();
    v_dist = -line_width * 0.5; gl_Position = vec4(ndc0 - offset, p0.z / p0.w, 1.0); EmitVertex();
    v_dist = line_width * 0.5; gl_Position = vec4(ndc1 + offset, p1.z / p1.w, 1.0); EmitVertex();
    v_dist = -line_width * 0.5; gl_Position = vec4(ndc1 - offset, p1.z / p1.w, 1.0); EmitVertex();
    EndPrimitive();
  }
)glsl";

static const GLchar *fragment_lines3d_source33 = R"glsl(
  #version 330 core
  out vec4 fragColor;

  uniform vec3 line_color;
  uniform float line_width;
  uniform float depth_offset;  // 深度オフセット（0.0001 など小さな値）
  
  in float v_dist;  // 線の中心からの距離

  void main() {
    // 線の中心からの距離を正規化（0.0が中心、1.0が端）
    float distance_from_center = abs(v_dist) / (line_width * 0.5);
    
    // アンチエイリアシング：境界付近でアルファ値を滑らかに変化させる
    float alpha = 1.0 - smoothstep(0.8, 1.0, distance_from_center);
    
    // シェーダーベースのポリゴンオフセット：深度値を少し手前に調整
    gl_FragDepth = gl_FragCoord.z - depth_offset;
    
    fragColor = vec4(line_color, alpha);
  }
)glsl";

//
// shaders for points 2d rendering
//

static const GLchar *vertex_points2d_source33 = R"glsl(
#version 330 core
layout(location = 0) in vec2 aPos;
uniform vec2 screenSize; // 画面サイズ（width, height）

void main() {
    // 座標を正規化デバイス座標系に変換
    vec2 normalizedPos = (aPos / screenSize) * 2.0 - 1.0;
    gl_Position = vec4(normalizedPos, 0.0, 1.0);
}
)glsl";

static const GLchar *geometry_points2d_source33 = R"glsl(
#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform float pointSize;     // NDC空間での半サイズ（縦横同じサイズ）
uniform vec2 screenSize;     // 画面サイズ

void main() {
    vec4 center = gl_in[0].gl_Position;
    
    // ピクセルサイズをNDC空間に変換
    float pixelSize = pointSize / screenSize.x * 2.0;
    vec2 offset = vec2(pixelSize);

    // 左下
    gl_Position = center + vec4(-offset.x, -offset.y, 0.0, 0.0);
    EmitVertex();

    // 右下
    gl_Position = center + vec4( offset.x, -offset.y, 0.0, 0.0);
    EmitVertex();

    // 左上
    gl_Position = center + vec4(-offset.x,  offset.y, 0.0, 0.0);
    EmitVertex();

    // 右上
    gl_Position = center + vec4( offset.x,  offset.y, 0.0, 0.0);
    EmitVertex();

    EndPrimitive();
}
)glsl";

static const GLchar *fragment_points2d_source33 = R"glsl(
#version 330 core
out vec4 FragColor;
uniform vec3 pointColor;

void main() {
    FragColor = vec4(pointColor, 1.0);
}
)glsl";

//
// shaders for lines 2d rendering
//
//

static const GLchar *vertex_lines2d_source33 = R"glsl(
  #version 330 core
  layout(location = 0) in vec2 position;

  uniform vec2 viewport_size;

  void main() {
    // 画面座標 → NDC (-1〜1)
    vec2 ndc = (position / viewport_size) * 2.0 - 1.0;
    gl_Position = vec4(ndc, 0.0, 1.0);
  }
)glsl";

static const GLchar *geometry_lines2d_source33 = R"glsl(
  #version 330 core
  layout(lines) in;
  layout(triangle_strip, max_vertices = 4) out;

  uniform float line_width;    // ピクセル単位
  uniform vec2 viewport_size;  // 画面サイズ (width, height)

  void main() {
    // NDC -> screen space
    vec2 p0 = ((gl_in[0].gl_Position.xy / gl_in[0].gl_Position.w) * 0.5 + 0.5) * viewport_size;
    vec2 p1 = ((gl_in[1].gl_Position.xy / gl_in[1].gl_Position.w) * 0.5 + 0.5) * viewport_size;

    vec2 dir = normalize(p1 - p0);
    vec2 normal = vec2(-dir.y, dir.x);
    vec2 offset = normal * line_width * 0.5;

    // Screen space → NDC
    vec2 o_ndc = offset / viewport_size * 2.0;

    vec2 ndc0 = gl_in[0].gl_Position.xy / gl_in[0].gl_Position.w;
    vec2 ndc1 = gl_in[1].gl_Position.xy / gl_in[1].gl_Position.w;

    gl_Position = vec4(ndc0 + o_ndc, 0.0, 1.0); EmitVertex();
    gl_Position = vec4(ndc0 - o_ndc, 0.0, 1.0); EmitVertex();
    gl_Position = vec4(ndc1 + o_ndc, 0.0, 1.0); EmitVertex();
    gl_Position = vec4(ndc1 - o_ndc, 0.0, 1.0); EmitVertex();
    EndPrimitive();
  }
)glsl";

static const GLchar *fragment_lines2d_source33 = R"glsl(
  #version 330 core
  out vec4 fragColor;

  uniform vec3 line_color;

  void main() {
    fragColor = vec4(line_color, 1.0);
  }
)glsl";

#endif  // _SHADERS_H
