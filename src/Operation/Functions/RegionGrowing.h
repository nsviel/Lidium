#ifndef RG_H
#define RG_H

class Scene;
class Attribut;

#include "../../Parameters.h"

class RegionGrowing
{
public:
  //Constructor / Destructor
  RegionGrowing(Scene* scene);
  ~RegionGrowing();

public:
  void algo(Mesh* mesh);
  void algo_I_pcl(Mesh* mesh);
  void algo_N_pcl(Mesh* mesh);
  void algo_IN_pcl(Mesh* mesh);

  //Setters / Getters
  inline float* get_thres_dist(){return &thres_dist;}
  inline float* get_thres_ptColor(){return &thres_ptColor;}
  inline float* get_thres_regionColor(){return &thres_regionColor;}
  inline float* get_thres_minClusterSize(){return &thres_minClusterSize;}
  inline float* get_thres_Smoothness(){return &thres_Smoothness;}
  inline float* get_thres_Curvature(){return &thres_Curvature;}
  inline void set_RGoption_N(bool value){this->normalsON = value;}
  inline void set_RGoption_I(bool value){this->intensityON = value;}

private:
  Scene* sceneManager;
  Attribut* attribManager;

  float thres_dist;
  float thres_ptColor;
  float thres_regionColor;
  float thres_minClusterSize;
  float thres_maxClusterSize;
  float thres_Smoothness;
  float thres_Curvature;
  bool intensityON;
  bool normalsON;
};

#endif
