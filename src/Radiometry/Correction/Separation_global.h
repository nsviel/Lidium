#ifndef Separation_global_H
#define Separation_global_H

class Plotting;
class PolyRegression;
class BundleByClass;
class Reference;
class Ref_Operation;

#include "../../Parameters.h"

class Separation_global
{
public:
  //Constructor / Destructor
  Separation_global(Reference* refManager);
  ~Separation_global();

public:
  //Method functions
  bool algo(Mesh* mesh);

  //Sub-functions
  bool algo_angle();
  bool algo_distance();
  bool algo_correction(Mesh* mesh);

  //Math functions
  void compute_normalizeCoeffs_angle();
  void compute_normalizeCoeffs_distance();
  void compute_MeanError_angle();
  void compute_MeanError_distance();
  void compute_Reference_angle();
  void compute_Reference_distance();

  //Plotting functions
  void plot_IbyR();
  void plot_IbyR_atomic();
  void plot_IbyIt();
  void plot_MeanError();
  void plot_bundleByClass(Mesh* mesh);
  void plot_PolyRegression(Mesh* mesh);
  void plot_Icorr(Mesh* mesh);

  //Setters / Getters
  inline vector<float> get_Im(){return Im;}
  inline vector<float> get_Ic(){return Ic;}

private:
  BundleByClass* bundler;
  Plotting* plotManager;
  Reference* refManager;
  Ref_Operation* refopeManager;

  int R_n, cIt_n;
  bool matlab_coeffs;
  bool verbose;
  vector<float> Im, Ic;
  vector<float> R_curve, cIt_curve;
  vector<float> rho_coeffs, R_coeffs, cIt_coeffs;
  vector<float> R_coeffs_normalized, cIt_coeffs_normalized;
  vector<float> cIt_lambda, R_lambda;
  vector<float> coeff_99, coeff_50, coeff_25, coeff_10;
  vector<float> ebyn_R_n, ebyn_cIt_n;
  vector<float> ebyn_R_e, ebyn_cIt_e;
};

#endif
