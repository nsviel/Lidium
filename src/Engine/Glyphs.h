#ifndef GLYPHS_H
#define GLYPHS_H

/**
 * \file Glyphs.h
 * \brief Glyph management
 * \author Nathan Sanchiz-Viel
 *
 * Deal with specific background objects (grid, axis ...)
 *
 */

class Loader;
class Transforms;

#include "../Parameters.h"

class Glyphs
{
public:
  //Constructor / Destructor
  Glyphs();
  ~Glyphs();

public:
  void init();
  void drawing();
  void clear();
  void reset();
  void reset_colors();

  //Glyph update
  void update(Mesh* mesh);
  void update_MinMaxCoords(Glyph* glyph);
  void update_location(Glyph* glyph);
  void update_color(Glyph* glyph);
  void update_color(Glyph* glyph, vec3 RGB);

  //Glyph declaration
  void obj_grid();
  void obj_subgrid();
  void obj_planegrid();
  void obj_axis();
  void obj_cube();
  void obj_normals(Mesh* mesh);
  void obj_axisMesh(Mesh* mesh);
  void obj_aabb(Mesh* mesh);
  void obj_matching(Mesh* mesh_1, Mesh* mesh_2);
  void obj_axisCircle(float circleRadius);
  string obj_pointsAtLocation(vector<vec3>& pos, float r, vec4 color);
  string obj_sphere_RGB(double r, int lats, int longs, vec3 pos, vec3 color);

  //Glyph functions
  void removeGlyph(int ID);
  void removeGlyph(string ID);
  void changeColor(int ID, vec3 RGB);
  void declareGlyph(string name, string type, float width, bool visible);
  void set_visibility(string name, bool value);
  int loadGlyph(string path, vec3 pos, string mode, bool perma);
  int loadGlyph(string path, vec3 pos, string mode, bool perma, int size);
  int loadGlyph_Plane(vec4 coeffs, string mode, bool perma);
  int get_glyphID(string name);
  int createGlyph(vector<vec3>& XYZ, vector<vec4>& RGB, string mode, bool perma);
  void create_Plane(vector<vec3>& XYZ, vector<vec4>& RGB, int SIZE);
  Glyph* get_glyphByName(string name);
  Glyph* get_glyphByID(int ID);
  bool is_glyphByName(string name);
  bool is_glyphVisible(string name);

  inline void set_normalSize(int value){this->normalSize = value;}
  inline void set_matching_rdm_color(bool value){this->match_rdmColor = value; upColor = true;}
  inline vec3 get_normalColor(){return normalColor;}
  inline vec3 get_gridColor(){return gridColor;}
  inline vec3 get_aabbColor(){return aabbColor;}
  inline vec3 get_matchingColor(){return matchingColor;}
  inline bool* get_aabbVisibility(){return &aabbON;}
  inline bool* get_matchVisibility(){return &matchON;}

private:
  Transforms* transformManager;
  Loader* loaderManager;

  list<Glyph*>* list_Glyph;
  vec3 normalColor, aabbColor, gridColor;
  vec3 subgridColor, planegridColor, matchingColor;
  bool match_rdmColor, upColor;
  bool aabbON, matchON;
  int ID, ID_sphere, ID_points;
  int normalSize, pointSize;
  int gridNbCells;
};

#endif
