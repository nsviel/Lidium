#ifndef FILTER_H
#define FILTER_H

class Scene;
class Attribut;

#include "../Parameters.h"

class Filter
{
public:
  //Constructor / Destructor
  Filter(Scene* scene);
  ~Filter();

public:
  void randSampling(Mesh* mesh);
  void outlierRemoval(Mesh* mesh);
  void statisticalRemoval(Mesh* mesh);
  void spaceSampling_PCL(Mesh* mesh);
  void spaceSampling(Mesh* mesh, float resolution);
  void filterByAngle(Mesh* mesh, float sampleAngle);
  void sphereCleaning_all();

  //Setters / Getters
  inline void set_sampling(int value){this->samplingPercent = value;}
  inline void set_squareSizeSampling(float value){this->squareSizeSampling = value;}
  inline void set_outRadiusSearch(float value){this->outRadiusSearch = value;}
  inline void set_samplingstd(float value){this->sampling_std = value;}
  inline void set_sphereDiameter(float value){this->sphereDiameter = value;}

private:
  Scene* sceneManager;
  Attribut* attribManager;

  int samplingPercent;
  float squareSizeSampling;
  float outRadiusSearch;
  float sampling_std;
  float sphereDiameter;
};

#endif
