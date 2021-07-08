#ifndef ENGINE_H
#define ENGINE_H

/**
 * \file Engine.h
 * \brief Data processing unit
 * \author Nathan Sanchiz-Viel
 *
 * Take responsability of class objects management and drawing stuff
 *
 */

class Scene;
class Attribut;
class Operation;
class Glyphs;
class Filter;
class Shader;
class Textures;
class Camera;
class HeatMap;
class Extraction;
class Selection;
class RegionGrowing;
class Registration;
class Radiometry;
class CoordTransform;
class Dimension;
class GUI;

#include "../Parameters.h"

class Engine
{
public:
  //Constructor / Destructor
  Engine(Dimension* dim, Shader* shader, Camera* control);
  ~Engine();

public:
  //Program functions
  void init();
  void loop();

  //Subfunctions
  void draw_things();
  void draw_clouds();

  inline Scene* get_SceneManager(){return sceneManager;}
  inline Glyphs* get_glyphManager(){return glyphManager;}
  inline Registration* get_regisManager(){return regisManager;}
  inline Filter* get_FilterManager(){return filterManager;}
  inline Attribut* get_AttribManager(){return attribManager;}
  inline Operation* get_OpeManager(){return opeManager;}
  inline HeatMap* get_heatmapManager(){return heatmapManager;}
  inline Radiometry* get_RadioManager(){return radioManager;}
  inline Extraction* get_extractionManager(){return extractionManager;}
  inline Selection* get_selectionManager(){return selectionManager;}
  inline RegionGrowing* get_growingManager(){return growingManager;}
  inline Textures* get_texManager(){return texManager;}
  inline Dimension* get_dimManager(){return dimManager;}
  inline Camera* get_CameraManager(){return cameraManager;}
  inline void set_pointSize(int value){this->pointSize = value;}

private:
  Dimension* dimManager;
  Shader* shaderManager;
  Scene* sceneManager;
  Registration* regisManager;
  Attribut* attribManager;
  Operation* opeManager;
  Radiometry* radioManager;
  Filter* filterManager;
  Extraction* extractionManager;
  HeatMap* heatmapManager;
  Glyphs* glyphManager;
  Selection* selectionManager;
  RegionGrowing* growingManager;
  Textures* texManager;
  Camera* cameraManager;
  CoordTransform* coordTransManager;
  GUI* guiManager;

  int pointSize;
  uint modelID, comID;
  vec3 backgColor;
};

#endif
