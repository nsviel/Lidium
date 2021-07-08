#ifndef HEATMAP_H
#define HEATMAP_H

class Scene;
class Attribut;

#include "../../Parameters.h"

class HeatMap
{
public:
  //Constructor / Destructor
  HeatMap(Scene* scene);
  ~HeatMap();

public:
  //Main functions
  void set_HeatMap_all(bool heatAll);
  void set_HeatMap(Mesh* mesh);
  void set_HeatMapField(int value);

  //Heatmap functions
  void compute_applyHeatmap(Mesh* mesh);
  void compute_reverseHeatmap(Mesh* mesh);
  void compute_heatmapColor(Mesh* mesh, vector<float>& v_in);

  //Plot functions
  void plot_colorPalette(Mesh* mesh);

  //Setters / Getters
  inline void set_normalized(bool value){this->normalized = value;}

private:
  Scene* sceneManager;
  Attribut* attribManager;

  int HMmode;
  bool normalized;
};

#endif
