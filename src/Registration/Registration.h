#ifndef Registration_H
#define Registration_H

class Scene;
class Glyphs;
class Transforms;
class ICP;
class Attribut;
class Algo_PCL;
class Plotting;

#include "../Parameters.h"

class Registration
{
public:
  //Constructor / Destructor
  Registration(Scene* scene, Glyphs* glyph);
  ~Registration();

public:
  //Main functions
  void make_Iteration();
  void make_algoPCL(int method);

  //Subfunctions
  void reset();
  void restart();
  void print_ICP();
  void colorization(Mesh* mesh_P, Mesh* mesh_Q);

  //Accesseurs
  inline ICP* get_icpManager(){return icpManager;}
  inline vector<float> get_vec_SSE(){return vec_SSE;}
  inline vector<int> get_vec_iter(){return vec_iter;}
  inline vector<vec3> get_vec_trans(){return vec_trans;}
  inline vector<vec3> get_vec_rotat(){return vec_rotat;}
  inline float get_duration(){return duration;}
  inline int get_iter(){return nbIter;}
  inline int* get_nbIter_max(){return &nbIter_max;}
  inline int* get_colorMethode(){return &colorMeth;}
  inline float* get_SSE_max(){return &SSE_max;}

private:
  Scene* sceneManager;
  Attribut* attribManager;
  Glyphs* glyphManager;
  ICP* icpManager;
  Algo_PCL* pclManager;
  Plotting* plotManager;

  vector<float> vec_SSE;
  vector<int> vec_iter;
  vector<vec3> vec_trans, vec_rotat;
  float duration;
  float SSE, SSE_max, SSE_mean;
  int nbIter, nbIter_max;
  int colorMeth;
};

#endif
