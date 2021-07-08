#ifndef METHOD_SIMPLIFIEDSURFACIC_H
#define METHOD_SIMPLIFIEDSURFACIC_H

class Plotting;
class PolyRegression;
class BundleByClass;
class Reference;

#include "../../Parameters.h"

class Surfacic_simplified
{
public:
  //Constructor / Destructor
  Surfacic_simplified(Reference* refClass);
  ~Surfacic_simplified();

public:
  //Method functions
  bool algo_Bretagne(Mesh* mesh);

  //Sub-functions
  bool compute_linearRegression();
  bool compute_quadraRegression();
  bool compute_correction(Mesh* mesh);
  bool compute_IIR2(vector<float>& Ib, vector<float>& Ib_dist);

  //Plotting
  void plot_bundleByClass(Mesh* mesh);
  void plot_linearRegression(Mesh* mesh);
  void plot_quadraticRegression(Mesh* mesh);
  void plot_intensityCorrection(Mesh* mesh);

  //Setters / Getters
  inline vector<float> get_Im(){return Im;}
  inline vector<float> get_Ic(){return Ic;}

private:
  BundleByClass* bundler;
  Plotting* plotManager;
  Reference* refManager;

  vector<float> lin_A, lin_B, mdist;
  vector<float> quad_A, quad_B;
  vector<float> Im, Ic;
  int nA, nB;
};

#endif
