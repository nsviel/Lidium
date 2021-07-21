#ifndef ATTRIBUT_H
#define ATTRIBUT_H

class Scene;

#include "../Parameters.h"

class Attribut
{
public:
  //Constructor / Destructor
  Attribut(Scene* scene);
  ~Attribut();

public:
  //General
  void compute_meshAttributs_all();
  void compute_meshAttributs_list(list<Mesh*>* list);
  void compute_meshAttributs(Mesh* mesh);
  void compute_distToScanner(Mesh* mesh);
  void compute_Distances(Mesh* mesh);
  void make_supressPoints(Mesh* mesh, vector<int>& idx);
  void make_supressPoints(vector<vec3>& XYZ, vector<int>& idx);
  void make_supressPoint(Mesh* mesh, int id);
  void cloudsData();

  //Color
  void set_pointCloudColor(Mesh* mesh, vec4 RGBA);
  void set_enhancedColor(Mesh* mesh);
  void set_randomizeAllPointsColor(Mesh* mesh);
  void set_restoreInitialColor(Mesh* mesh);
  void set_colorRGB_all();
  void set_colorRGB(Mesh* mesh);
  void set_colorI_all();
  void set_colorI(Mesh* mesh);

  //Normal
  void compute_normals(Mesh* mesh);
  void compute_normals_PCL(Mesh* mesh);
  void compute_normals_Hough(Mesh* mesh);
  void compute_normals_sphere(Mesh* mesh);
  void compute_normals_invert();
  void compute_normals_reorientToOrigin(Mesh* mesh);
  void compute_normals_planXaxis(Mesh* mesh);
  void compute_normals_planYaxis(Mesh* mesh);
  void compute_normals_planZaxis(Mesh* mesh);
  void compute_normals_planFitting(Mesh* mesh);

  void compute_cosIt(Mesh* mesh);
  void compute_checkForNan(Mesh* mesh);

  //Intensity
  void compute_intensityInversion();
  void compute_colorToIntensity(Mesh* mesh);
  void fct_convert255to2048(Mesh* mesh);
  void fct_convert2048to255(Mesh* mesh);
  void fct_moins();
  void fct_IsRange(vec2 range);
  vec2 get_IsRange();

  inline float get_sphereRadius(){return sphereRadius;}
  inline void set_normalRadiusSeach(float value){this->radiusSearch = value;}
  inline void set_sphereRadius(float value){this->sphereRadius = value;}

private:
  Scene* sceneManager;

  float radiusSearch;
  float sphereRadius;
};

#endif
