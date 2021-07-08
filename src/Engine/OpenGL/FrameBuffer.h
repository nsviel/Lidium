#ifndef DEF_FRAMEBUFFER
#define DEF_FRAMEBUFFER

#include "../../Parameters.h"

class FrameBuffer
{
public:
  FrameBuffer();
  ~FrameBuffer();

public:
  void display();
  void create_FBO();
  void create_Texture();
  void create_DepthRenderBuffer();
  void create_bckgrndQuad();
  void readScreenPixels(int width, int height);

private:
  GLuint idFBO;
  GLuint idTexture;
  GLuint idDepthrenderbuffer;
  int width, height;
};

#endif
