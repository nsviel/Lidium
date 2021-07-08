#ifndef RadarEquation_H
#define RadarEquation_H

class Plotting;

#include "../../Parameters.h"

class RadarEquation
{
public:
  //Constructor / Destructor
  RadarEquation();
  ~RadarEquation();

public:
  //Method functions
  void compute_RadarEquation(Mesh* mesh);
  void compute_IR2bycosIt(Mesh* mesh);
  void compute_IbyR2(Mesh* mesh);
  void compute_IbyCosIt(Mesh* mesh);

  //Plotting
  void plot_intensityCorrected(Mesh* mesh);

  //Setters / Getters
  inline vector<float> get_Ic(){return Ic;}

private:
  Plotting* plotManager;

  vector<float> Ic;
  float D;
  float lambda;
  float Pe;
  float R_ref;
};

#endif
