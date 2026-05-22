////////////////////////////////////////////////////////////////////
//
// $Id: stb_util.hxx 2021/06/13 15:20:47 kanai Exp $
//
// Copyright (c) 2021 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _STB_UTIL_HXX
#define _STB_UTIL_HXX 1

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <vector>

#include "GLPanel.hxx"

class mySTB {

public:

  int stb_load_texture( const char* const filename, GLPanel& pane ) {
    int w, h, comp;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* image = stbi_load(filename, &w, &h, &comp, STBI_rgb);
    if (image == nullptr)
      throw(std::string("Failed to load texture"));

    const int channels = 3;
    std::vector<unsigned char> pixels(image,
                                      image + static_cast<size_t>(w) * h * channels);
    stbi_image_free(image);

    while (::glGetError() != GL_NO_ERROR) {}

    GLuint tex = 0;
    ::glGenTextures(1, &tex);
    ::glBindTexture(GL_TEXTURE_2D, tex);
    ::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    ::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE,
                   pixels.data());
  GLenum err = ::glGetError();
  if (err != GL_NO_ERROR) {
    throw std::string("glTexImage2D failed: ") + std::to_string(err);
  }
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    ::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    ::glGenerateMipmap(GL_TEXTURE_2D);
    ::glBindTexture(GL_TEXTURE_2D, 0);

    return static_cast<unsigned int>(tex);
  };

  void stb_capture_and_write( const char* filename, int w, int h, int c, GLPanel& pane )
  {
    std::vector<unsigned char> img( w*h*c );
    pane.capture( img, w, h, c );
    // stbi_flip_vertically_on_write(true); // no effect in this path
    stbi_write_png( filename, w, h, c, &(img[0]), w*c );
    std::cout << "save " << w << "x" << h << " png image <" << filename << "> done. " << std::endl;
  };

};

#endif // STB_UTIL_HXX
