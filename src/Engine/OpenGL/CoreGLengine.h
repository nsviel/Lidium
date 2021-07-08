#ifndef DEF_SCENEOPENGL
#define DEF_SCENEOPENGL

class Dimension;
class Shader;
class Camera;
class GUI;
class Engine;
class Configuration;

#include "../../Parameters.h"

class CoreGLengine
{
public:
  CoreGLengine();
  ~CoreGLengine();

public:
  bool init();
  bool init_OGL();
  bool init_object();
  bool init_shader();

  //GL loop
  void loop();
  void loop_begin();
  void loop_end();

private:
  Configuration* configManager;
  GLFWwindow* window;
  Shader* shaderManager;
  Engine* engineManager;
  Camera* cameraManager;
  Dimension* dimManager;
  GUI* guiManager;

  uint shaderID, mvpID;
  vec3 backgColor;
};

#endif
