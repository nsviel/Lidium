#ifndef Fitting_H
#define Fitting_H

class Scene;
class Glyphs;
class Attribut;

#include "../../Parameters.h"

class Fitting
{
public:
  Fitting(Scene* scene);
  ~Fitting();

public:
  //Sphere fitting
  void Sphere_MeshToCenter_all();
  void Sphere_MeshToCenter(Mesh* mesh);
  vec3 Sphere_FindCenter(Mesh* mesh);

  //Plane fitting
  void Plane_Mesh_all();
  void Plane_Mesh(Mesh* mesh);

private:
  Scene* sceneManager;
  Glyphs* glyphManager;
  Attribut* attribManager;

  float Radius;
};
#endif
