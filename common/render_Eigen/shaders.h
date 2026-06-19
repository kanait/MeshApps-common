////////////////////////////////////////////////////////////////////
//
// $Id: shaders.h 2026/04/30 18:01:29 tkskn Exp 
//
// Copyright (c) 2021-2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _SHADERS_H
#define _SHADERS_H 1

//
// for OpenGL 3.3
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

// Four-light version (point/directional lights + constant weights)
static const GLchar *fragment_shader_Phong_source33 = R"glsl(
#version 330 core

in vec4 frag_position;
in vec3 frag_normal;

uniform vec4 light_position[4];     // Distinguish point/directional by w
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

    // Per-light intensity factors (Key, Fill, Rim, Ambient-like)
    //float light_weights[4] = float[4](0.7, 0.3, 0.5, 0.2);
    float light_weights[4] = float[4](0.56, 0.24, 0.40, 0.16); // 80% of original

    for (int i = 0; i < 4; ++i) {
      if (!light_enabled[i]) continue;

      vec3 lightDir;
      if (light_position[i].w == 0.0) {
        // Directional light (direction)
        lightDir = normalize(-light_position[i].xyz);
      } else {
        // Point light (position)
        lightDir = normalize(light_position[i].xyz - frag_position.xyz);
      }

      vec3 halfwayDir = normalize(lightDir + viewDir);

      // Shadow-like effect
      float ndotl = dot(normal, lightDir);
      float shadow = ndotl > 0.0 ? 1.0 : 0.1; // pseudo-shadow term

      float diff = max(ndotl, 0.0);
      float spec = pow(max(dot(normal, halfwayDir), 0.0), shininess);

      // Simple AO
      //float ao = pow(1.0 - abs(dot(normal, vec3(0.0, 1.0, 0.0))), 2.0);  // occlusion amount from above
      //vec3 ambient = ambient_color * mix(0.3, 1.0, ao);
      vec3 ambient  = ambient_color;
      vec3 diffuse  = diffuse_color * diff * shadow;
      vec3 specular = specular_color * spec * shadow;

      // Accumulate with weights
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
  uniform vec2 viewport_size; // Screen resolution (pixels)
  uniform float line_width;   // Line width (in pixels)
  uniform float aspect;       // Screen aspect ratio (width / height)
  
  out float v_dist;  // Distance from line center (for anti-aliasing)

  void main()
  {
    // Projection transform
    vec4 p0 = projection * modelview * gl_in[0].gl_Position;
    vec4 p1 = projection * modelview * gl_in[1].gl_Position;

    // NDC conversion (clip space -> -1 to 1)
    vec2 ndc0 = p0.xy / p0.w;
    vec2 ndc1 = p1.xy / p1.w;

    // Line direction and its normal vector
    vec2 dir = normalize(ndc1 - ndc0);
    vec2 normal = vec2(-dir.y, dir.x);

    // Aspect ratio correction
    normal.x *= aspect;

    // Pixel -> NDC conversion: ratio to viewport width
    float pixel_size = 2.0 / viewport_size.x;
    vec2 offset = normal * line_width * 0.5 * pixel_size;

    // Emit 4 vertices composing a quad (preserve correct depth)
    // Compute distance from line center for anti-aliasing
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
  uniform float depth_offset;  // Depth offset (small value such as 0.0001)
  
  in float v_dist;  // Distance from line center

  void main() {
    // Normalize distance from line center (0.0 center, 1.0 edge)
    float distance_from_center = abs(v_dist) / (line_width * 0.5);
    
    // Anti-aliasing: smoothly vary alpha near boundaries
    float alpha = 1.0 - smoothstep(0.8, 1.0, distance_from_center);
    
    // Shader-based polygon offset: move depth slightly forward
    gl_FragDepth = gl_FragCoord.z - depth_offset;
    
    fragColor = vec4(line_color, alpha);
  }
)glsl";

//
// shaders for points 3d rendering
//

static const GLchar *vertex_points3d_source33 = R"glsl(
  #version 330 core
  layout(location = 0) in vec3 position;

  void main() {
    gl_Position = vec4(position, 1.0);
  }
)glsl";

static const GLchar *geometry_points3d_source33 = R"glsl(
  #version 330 core
  layout(points) in;
  layout(triangle_strip, max_vertices = 4) out;

  uniform mat4 modelview;
  uniform mat4 projection;
  uniform vec2 viewport_size;
  uniform float point_size;
  uniform float aspect;

  out float v_dist;

  void main()
  {
    vec4 p = projection * modelview * gl_in[0].gl_Position;
    vec2 ndc = p.xy / p.w;

    float pixel_size = 2.0 / viewport_size.x;
    vec2 point_half = vec2(point_size * 0.5 * pixel_size);
    point_half.x *= aspect;

    v_dist = point_size * 0.5;
    gl_Position = vec4(ndc + vec2(-point_half.x, -point_half.y), p.z / p.w, 1.0); EmitVertex();
    v_dist = -point_size * 0.5;
    gl_Position = vec4(ndc + vec2( point_half.x, -point_half.y), p.z / p.w, 1.0); EmitVertex();
    v_dist = point_size * 0.5;
    gl_Position = vec4(ndc + vec2(-point_half.x,  point_half.y), p.z / p.w, 1.0); EmitVertex();
    v_dist = -point_size * 0.5;
    gl_Position = vec4(ndc + vec2( point_half.x,  point_half.y), p.z / p.w, 1.0); EmitVertex();
    EndPrimitive();
  }
)glsl";

static const GLchar *fragment_points3d_source33 = R"glsl(
  #version 330 core
  out vec4 fragColor;

  uniform vec3 point_color;
  uniform float point_size;
  uniform float depth_offset;

  in float v_dist;

  void main() {
    float distance_from_center = abs(v_dist) / (point_size * 0.5);
    float alpha = 1.0 - smoothstep(0.8, 1.0, distance_from_center);
    gl_FragDepth = gl_FragCoord.z - depth_offset;
    fragColor = vec4(point_color, alpha);
  }
)glsl";

//
// shaders for points 2d rendering
//

static const GLchar *vertex_points2d_source33 = R"glsl(
#version 330 core
layout(location = 0) in vec2 aPos;
uniform vec2 screenSize; // Screen size (width, height)

void main() {
    // Convert coordinates to normalized device coordinates
    vec2 normalizedPos = (aPos / screenSize) * 2.0 - 1.0;
    gl_Position = vec4(normalizedPos, 0.0, 1.0);
}
)glsl";

static const GLchar *geometry_points2d_source33 = R"glsl(
#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform float pointSize;     // Half-size in NDC space (same in both axes)
uniform vec2 screenSize;     // Screen size

void main() {
    vec4 center = gl_in[0].gl_Position;
    
    // Convert pixel size to NDC space
    float pixelSize = pointSize / screenSize.x * 2.0;
    vec2 offset = vec2(pixelSize);

    // Bottom-left
    gl_Position = center + vec4(-offset.x, -offset.y, 0.0, 0.0);
    EmitVertex();

    // Bottom-right
    gl_Position = center + vec4( offset.x, -offset.y, 0.0, 0.0);
    EmitVertex();

    // Top-left
    gl_Position = center + vec4(-offset.x,  offset.y, 0.0, 0.0);
    EmitVertex();

    // Top-right
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
    // Screen coordinates -> NDC (-1 to 1)
    vec2 ndc = (position / viewport_size) * 2.0 - 1.0;
    gl_Position = vec4(ndc, 0.0, 1.0);
  }
)glsl";

static const GLchar *geometry_lines2d_source33 = R"glsl(
  #version 330 core
  layout(lines) in;
  layout(triangle_strip, max_vertices = 4) out;

  uniform float line_width;    // In pixels
  uniform vec2 viewport_size;  // Screen size (width, height)

  void main() {
    // NDC -> screen space
    vec2 p0 = ((gl_in[0].gl_Position.xy / gl_in[0].gl_Position.w) * 0.5 + 0.5) * viewport_size;
    vec2 p1 = ((gl_in[1].gl_Position.xy / gl_in[1].gl_Position.w) * 0.5 + 0.5) * viewport_size;

    vec2 dir = normalize(p1 - p0);
    vec2 normal = vec2(-dir.y, dir.x);
    vec2 offset = normal * line_width * 0.5;

    // Screen space -> NDC
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

//
// shaders for textured Phong shading
//

static const GLchar *vertex_shader_Texture_source33 = R"glsl(
#version 330 core

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_texcoord;

uniform mat4 modelview;
uniform mat4 projection;
uniform mat3 normalMatrix;

out vec4 frag_position;
out vec3 frag_normal;
out vec2 frag_texcoord;

void main()
{
    frag_position = modelview * vec4(in_position, 1.0);
    frag_normal = normalize(normalMatrix * in_normal);
    frag_texcoord = in_texcoord;
    gl_Position = projection * frag_position;
}
)glsl";

static const GLchar *fragment_shader_Texture_source33 = R"glsl(
#version 330 core

in vec4 frag_position;
in vec3 frag_normal;
in vec2 frag_texcoord;

uniform sampler2D texture_map;
uniform vec4 light_position[4];
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
    vec3 baseColor = texture(texture_map, frag_texcoord).rgb;
    vec3 finalColor = vec3(0.0);
    float light_weights[4] = float[4](0.56, 0.24, 0.40, 0.16);

    for (int i = 0; i < 4; ++i) {
      if (!light_enabled[i]) continue;

      vec3 lightDir;
      if (light_position[i].w == 0.0) {
        lightDir = normalize(-light_position[i].xyz);
      } else {
        lightDir = normalize(light_position[i].xyz - frag_position.xyz);
      }

      vec3 halfwayDir = normalize(lightDir + viewDir);
      float ndotl = dot(normal, lightDir);
      float shadow = ndotl > 0.0 ? 1.0 : 0.1;
      float diff = max(ndotl, 0.0);
      float spec = pow(max(dot(normal, halfwayDir), 0.0), shininess);

      vec3 ambient  = ambient_color * baseColor;
      vec3 diffuse  = diffuse_color * baseColor * diff * shadow;
      vec3 specular = specular_color * spec * shadow;

      finalColor += (ambient + diffuse + specular) * light_weights[i];
    }
    finalColor += emission_color;

    fragColor = vec4(finalColor, 1.0);
}
)glsl";

#endif  // _SHADERS_H
