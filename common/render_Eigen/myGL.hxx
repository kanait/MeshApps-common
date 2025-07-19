////////////////////////////////////////////////////////////////////
//
// $Id: myGL.hxx 2025/07/10 21:54:35 kanai Exp 
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _MYGL_HXX
#define _MYGL_HXX 1

#include <glad/glad.h>
#ifdef __APPLE__
    #include <OpenGL/gl.h>
#elif defined(WIN32) || defined(_WIN32) || defined(_WIN64)
    #include <GL/gl.h>
#else
    #include <GL/gl.h>
#endif

#endif // _MYGL_HXX
