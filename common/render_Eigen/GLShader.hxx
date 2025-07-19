////////////////////////////////////////////////////////////////////
//
// $Id: GLShader.hxx 2025/07/19 14:58:32 kanai Exp 
//
// Copyright (c) 2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _GLSHADER_HXX
#define _GLSHADER_HXX 1

#include "myGL.hxx"

#include "shaders.h"

struct GLShader {
  // Program
  GLuint phongShaderProgram=0;
  GLuint wireframeShaderProgram=0;
  GLuint gradShaderProgram=0;
  GLuint points2dShaderProgram=0;
  GLuint lines2dShaderProgram=0;
  GLuint lines3dShaderProgram=0;
  
  // Loc
  GLint projectionLoc = -1;
  GLint modelviewLoc = -1;
  GLint normalmatrixLoc = -1;
  GLint lightpositionLoc[4] = {-1,-1,-1,-1};
  GLint lightenabledLoc[4] = {-1,-1,-1,-1};
  GLint ambientcolorLoc = -1;
  GLint diffusecolorLoc = -1;
  GLint emissioncolorLoc = -1;
  GLint specularcolorLoc = -1;
  GLint shininessLoc = -1;
  // Loc for flat shading
  GLint wireframemodelviewLoc = -1;
  GLint wireframeprojectionLoc = -1;
  GLint lineWidthLoc = -1;
  // Loc for points 2d rendering
  GLint points2dScreenSizeLoc = -1;
  GLint points2dPointSizeLoc = -1;
  GLint points2dPointColorLoc = -1;
  // Loc for lines 2d rendering
  GLint lines2dScreenSizeLoc = -1;
  GLint lines2dLineWidthLoc = -1;
  GLint lines2dLineColorLoc = -1;
  // Loc for lines 3d rendering
  GLint lines3dModelViewLoc = -1;
  GLint lines3dProjectionLoc = -1;
  GLint lines3dViewportSizeLoc = -1;
  GLint lines3dLineWidthLoc = -1;
  GLint lines3dAspectLoc = -1;
  GLint lines3dLineColorLoc = -1;
  GLint lines3dDepthOffsetLoc = -1;
};

#endif // _GLSHADER_HXX 1
