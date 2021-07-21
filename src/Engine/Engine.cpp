#include "Engine.h"

#include "Scene.h"
#include "Glyphs.h"
#include "Dimension.h"
#include "Configuration.h"

#include "OpenGL/CoreGLengine.h"
#include "OpenGL/Shader.h"
#include "OpenGL/Textures.h"
#include "OpenGL/Camera.h"

#include "../Registration/Registration.h"
#include "../Radiometry/Radiometry.h"

#include "../Operation/Attribut.h"
#include "../Operation/Operation.h"
#include "../Operation/Functions/Heatmap.h"
#include "../Operation/Functions/Extraction.h"
#include "../Operation/Functions/CoordTransform.h"
#include "../Operation/Functions/Selection.h"
#include "../Operation/Functions/RegionGrowing.h"
#include "../Operation/Filter.h"

//Constructor / Destructor
Engine::Engine(Dimension* dim, Shader* shader, Camera* control){
  this->dimManager = dim;
  this->shaderManager = shader;
  this->cameraManager = control;
  //---------------------------

  this->glyphManager = new Glyphs();
  this->sceneManager = new Scene(glyphManager);
  this->coordTransManager = new CoordTransform(cameraManager, dimManager);
  this->selectionManager = new Selection(dimManager, sceneManager, glyphManager, cameraManager);
  this->attribManager = new Attribut(sceneManager);
  this->opeManager = new Operation(sceneManager,glyphManager);
  this->heatmapManager = new HeatMap(sceneManager);
  this->extractionManager = new Extraction(sceneManager);
  this->filterManager = new Filter(sceneManager);
  this->radioManager = new Radiometry(sceneManager);
  this->regisManager = new Registration(sceneManager, glyphManager);
  this->growingManager = new RegionGrowing(sceneManager);
  this->texManager = new Textures();
  //this->guiManager = new GUI(this, cameraManager);


  float backgColor = configuration.WINDOW_BckgColor;
  this->backgColor = vec3(backgColor, backgColor, backgColor);
  this->pointSize = 1;

  //---------------------------
}
Engine::~Engine(){
  //---------------------------

  delete sceneManager;
  delete attribManager;
  delete heatmapManager;
  delete extractionManager;
  delete filterManager;
  delete radioManager;
  delete glyphManager;
  delete regisManager;
  delete coordTransManager;

  //---------------------------
}

//Program functions
void Engine::loop(){
  Configuration configManager;
  //---------------------------

  this->draw_things();
  this->draw_clouds();

  //---------------------------
}
void Engine::Exit(){
  GLFWwindow* window = dimManager->get_window();
  glfwSetWindowShouldClose(window, true);
}

//Subfunctions
void Engine::draw_things(){
  //---------------------------

  vec3 camPos = cameraManager->get_camPos();
  glyphManager->drawing();
  selectionManager->update();

  //---------------------------
}
void Engine::draw_clouds(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  glPointSize(pointSize);
  for(int i=0;i<list_Mesh->size();i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    //Vertices
    glBindVertexArray(mesh->VAO);
    glDrawArrays(GL_POINTS, 0, mesh->location.OBJ.size());
    glBindVertexArray(0);
  }

  //---------------------------
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(3);
}
