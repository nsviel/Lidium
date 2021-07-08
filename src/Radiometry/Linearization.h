#ifndef Linearization_H
#define Linearization_H

class Scene;
class Attribut;
class Reference;
class Ref_Operation;

#include "../Parameters.h"

class Linearization
{
public:
  //Constructor / Destructor
  Linearization(Scene* scene, Ref_Operation* ope);
  ~Linearization();

public:
  //General functions
  void algo_linearization(Mesh* mesh, int method);
  void algo_reverse(Mesh* mesh, int method);

  //Linearization
  void lin_SurfacicGlobal(Mesh* mesh);
  void lin_SeparationGlobal(Mesh* mesh);
  void lin_SurfacicLocal(Mesh* mesh);

  //Reverse
  void rev_SurfacicGlobal(Mesh* mesh);
  void rev_SeparationGlobal(Mesh* mesh);

  //Math functions
  void compute_SpectralonValues();
  void compute_gammaCoefficients();

  //Setters / Getters
  inline void set_mode(int value){this->mode = value;}

private:
  Scene* sceneManager;
  Ref_Operation* opeManager;
  Reference* refManager;
  Attribut* attribManager;

  float gamma_C, gamma_G, gamma_D;
  int mode;
};

#endif
