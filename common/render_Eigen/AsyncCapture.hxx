////////////////////////////////////////////////////////////////////
//
// $Id: AsyncCapture.hxx 2025/07/18 01:26:54 kanai Exp 
//
// Copyright (c) 2025 Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _ASYNCCAPTURE_HX
#define _ASYNCCAPTURE_HXX 1

#include "myGL.hxx"
#include <vector>
#include <iostream>

class AsyncCapture {
public:
  AsyncCapture(int w, int h, int channels)
    : width(w), height(h), channelCount(channels), index(0) {
    bufferSize = width * height * channelCount;

    // 2つの PBO を作成（ダブルバッファリング）
    glGenBuffers(2, pbos);
    for (int i = 0; i < 2; ++i) {
      glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos[i]);
      glBufferData(GL_PIXEL_PACK_BUFFER, bufferSize, nullptr, GL_STREAM_READ);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    imageBuffer.resize(bufferSize);
  }

  ~AsyncCapture() {
    glDeleteBuffers(2, pbos);
  }

  // 呼び出すたびに1フレーム遅れの画像が取得できる
  void capture() {
    int nextIndex = (index + 1) % 2;

    // 今回の PBO に非同期で読み込む
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos[index]);
    glReadPixels(0, 0, width, height,
                 (channelCount == 4) ? GL_RGBA : GL_RGB,
                 GL_UNSIGNED_BYTE, 0); // PBO へ直接書き込む

    // 1フレーム遅れで読み出す（前回の PBO から）
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos[nextIndex]);
    void* ptr = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
    if (ptr) {
      std::memcpy(imageBuffer.data(), ptr, bufferSize);
      glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

      // 画像は上下反転されているので flip
      flipVertically(imageBuffer);
    }

    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    index = nextIndex;
  }

  const std::vector<unsigned char>& getImage() const {
    return imageBuffer;
  }

private:
  int width, height, channelCount, bufferSize;
  GLuint pbos[2];
  int index; // 今回の書き込み用インデックス
  std::vector<unsigned char> imageBuffer;

  void flipVertically(std::vector<unsigned char>& data) {
    int rowSize = width * channelCount;
    std::vector<unsigned char> tempRow(rowSize);
    for (int y = 0; y < height / 2; ++y) {
      int top = y * rowSize;
      int bottom = (height - 1 - y) * rowSize;
      std::memcpy(tempRow.data(), &data[top], rowSize);
      std::memcpy(&data[top], &data[bottom], rowSize);
      std::memcpy(&data[bottom], tempRow.data(), rowSize);
    }
  }
};

#endif // _ASYNCCAPTURE_HX
