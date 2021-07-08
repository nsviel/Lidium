#ifndef Selection_H
#define Selection_H

class Scene;
class Glyphs;
class Transforms;
class Attribut;
class Camera;
class Dimension;
class CoordTransform;

#include "../../Parameters.h"

class Selection
{
public:
  //Constructor / Destructor
  Selection(Dimension* dim, Scene* scene, Glyphs* glyph, Camera* control);
  ~Selection();

public:
  //Drawing functions
  void update();
  void validate();

  //Point selection
  void selectionPoint(vec3 point);
  void mark_pointCreation(vec3 point);
  bool mark_pointSupression(vec3 point);
  void mark_pointColor(Mesh* sphere, int num);
  void mark_pointLocation();
  void mark_supressAll();
  bool mark_supressSelectedPoints_all();
  void mark_supressSelectedPoints(Mesh* mesh);

  //Mouse interactivity
  void mouse_selection(int selectMouseMode);
  void mouse_frameSelection(vec2 pt1, vec2 pt2);
  void mouse_drawFrame(vec2 point1, vec2 point2);
  void mouse_meshPicking();
  vec3 mouse_cameraPt();
  vec3 mouse_clickedPoint();
  void update_glDims();

  //Plane
  void mark_planeCreation();
  void mark_planeABpoints(Mesh* mesh);
  void mark_planeLocation();

  //Spectralon
  void Spectralon_extractionProcess(Mesh* mesh);
  void Spectralon_extractSpectralon(Mesh* mesh);
  void Spectralon_computeAttibutes(Mesh* mesh);
  void Spectralon_extractParts(Mesh* mesh);
  void Spectralon_extractParts_generic(Mesh* mesh, string percent, float X_min, float x_max, float x_mean);
  void Spectralon_ABpoints(Mesh* mesh);

  //Setters / Getters
  inline void set_markMode(string mode){this->markMode = mode;}
  inline void set_selectionSensibility(float value){this->selectSensibility = value;}
  inline void set_guiX(float value){this->gui_X = value;}
  inline void set_guiY(float value){this->gui_Y = value;}
  inline void set_glX(float value){this->gl_X = value;}
  inline void set_glY(float value){this->gl_Y = value;}

private:
  Dimension* dimManager;
  Scene* sceneManager;
  Attribut* attribManager;
  Transforms* transformManager;
  Glyphs* glyphManager;
  Camera* controlsManager;
  CoordTransform* coordTransManager;

  list<Mesh*> list_Mark;
  list<int> list_glyph;
  Mesh* planeMark;
  Mesh* Spectralon;
  vec3 A, B, C, D, E, F;
  string markMode;
  float selectSensibility;
  float angle;
  float gui_X, gui_Y, gl_X, gl_Y;
  int nbMark;
  int ID_plane;
};

#endif
