#include "CoreGLengine.h"

#include "Shader.h"
#include "Camera.h"
#include "../Dimension.h"
#include "../../GUI/GUI.h"
#include "../Engine.h"
#include "../Configuration.h"

#define GLM_ENABLE_EXPERIMENTAL
#define GL_GLEXT_PROTOTYPES

//Constructor / Destructor
CoreGLengine::CoreGLengine(){
  //---------------------------

  this->configManager = new Configuration();

  float backgColor = configuration.WINDOW_BckgColor;
  this->backgColor = vec3(backgColor, backgColor, backgColor);

  //---------------------------
}
CoreGLengine::~CoreGLengine(){
  //---------------------------

  glDeleteProgram(shaderID);
  glfwDestroyWindow(window);
  glfwTerminate();

  //---------------------------
}

//Main loop
bool CoreGLengine::init(){
  this->configManager->make_configuration();
  //---------------------------

  this->init_OGL();
  this->init_shader();
  this->init_object();

  //---------------------------
  return true;
}
bool CoreGLengine::init_OGL(){
  int gl_width = configuration.WINDOW_InitResWidth - configuration.GUI_LeftPanel_width;
  int gl_height = configuration.WINDOW_InitResHeight - configuration.GUI_TopPanel_height;
  //---------------------------

  //GLFW
  glfwInit();
  if(configuration.GL_ForceVersion){
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,6);
  }
  glfwWindowHint(GLFW_SAMPLES, configuration.WINDOW_MultiSample);
  glfwWindowHint(GLFW_OPENGL_CORE_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  window = glfwCreateWindow(gl_width,gl_height,"Interface",NULL,NULL);
  if(window == NULL){
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return false;
  }else if(configuration.VERBOSE_coreGL){;
    std::cout << "GLFW window created" << std::endl;
  }

  glfwMakeContextCurrent(window);
  glfwSetInputMode(window,GLFW_STICKY_KEYS,GL_TRUE);
  glfwSetWindowTitle(window, configuration.WINDOW_Title);

  //GL
  glViewport(0, 0, gl_width, gl_height/2);
  glPointSize(1);
  glLineWidth(1);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthFunc(GL_LESS);

  //GLEW
  glewInit();
  if(configuration.VERBOSE_coreGL){
    std::cout << "GLEW initiated" << std::endl;
  }

  //---------------------------
  return true;
}
bool CoreGLengine::init_shader(){
  this->shaderManager = new Shader();
  //---------------------------

  shaderManager->shaderManagment("../src/Engine/shaders/");
  shaderID = shaderManager->get_programID();
  mvpID = shaderManager->get_mvpID();
  glUseProgram(shaderID);
  if(configuration.VERBOSE_coreGL){
    std::cout << "Shaders created" << std::endl;
  }

  //---------------------------
  return true;
}
bool CoreGLengine::init_object(){
  //---------------------------

  this->dimManager = new Dimension(window);
  this->cameraManager = new Camera(dimManager);
  this->engineManager = new Engine(dimManager, shaderManager, cameraManager);
  this->guiManager = new GUI(engineManager);
  guiManager->Gui_bkgColor(&backgColor);

  //---------------------------
  return true;
}

//GL loop
void CoreGLengine::loop(){
  //---------------------------

  do{
    this->loop_begin();

    //Viewport
    dimManager->update_window_dim();
    vec2 glDim = dimManager->get_glDim();
    vec2 glPos = dimManager->get_glPos();
    cameraManager->viewport(glPos, glDim);
    cameraManager->input_cameraMouseCommands();
    cameraManager->input_cameraKeyCommands();

    //Shader
    mat4 mvp = cameraManager->compute_mvpMatrix();
    glUniformMatrix4fv(mvpID, 1, GL_FALSE, &mvp[0][0]);

    //Engine
    engineManager->loop();
    guiManager->Gui_loop();

    this->loop_end();
  }
  while(!glfwWindowShouldClose(window));

  //---------------------------
  configManager->save_configuration();
}
void CoreGLengine::loop_begin(){
  //---------------------------

  glfwPollEvents();
  glClearColor(backgColor.x, backgColor.y, backgColor.z, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //---------------------------
}
void CoreGLengine::loop_end(){
  //---------------------------

  glfwSwapBuffers(window);
  if(configuration.GL_WaitForEvent) glfwWaitEvents();

  //---------------------------
}
