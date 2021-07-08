#ifndef Surfacic_globalPiecewise_H
#define Surfacic_globalPiecewise_H

class Ref_Operation;

#include "../../Parameters.h"

class Surfacic_globalPiecewise
{
public:
  //Constructor / Destructor
  Surfacic_globalPiecewise(Ref_Operation* opeClass);
  ~Surfacic_globalPiecewise();

public:
  //Method function
  bool algo(Mesh* mesh);

  //Sub-functions
  bool algo_nbP();
  bool algo_fitting();
  bool algo_spaceParameter();
  bool algo_correction(Mesh* mesh);

  //Surface fitting
  float multivariatePoly(VectorXf P, float x, float y);
  MatrixXf compute_Jacobian(vector<float> R, vector<float> cIt);
  VectorXf compute_SurfaceRegression(MatrixXf J, vector<float> R, vector<float> cIt, vector<float> I);

  //Specific functions
  bool compute_ReferencePoints(VectorXf P_b, VectorXf P_m, VectorXf P_e);
  void compute_normalizeCoeffs(VectorXf P);
  void compute_error(Mesh* mesh);

  //Plotting
  void plot_SurfaceFitting();
  void plot_intensityCorrection(Mesh* mesh);
  void plot_SpaceParameter();

  //Setters / Getters
  inline void set_m(int value){this->m = value;}
  inline void set_n(int value){this->n = value;}
  inline vector<float> get_Im(){return Im;}
  inline vector<float> get_Ic(){return Ic;}
  inline float* get_segment_1(){return &Segment_1;}
  inline float* get_segment_2(){return &Segment_2;}

private:
  Ref_Operation* refopeManager;

  int m, n, nbP;
  float Segment_1, Segment_2;
  float lambda_b, lambda_m, lambda_e;

  vector<float> Im, Ic;
  vector<float> I01_tot, R_tot, cIt_tot;
  vector<float> I2048_beg, R_beg, cIt_beg;
  vector<float> I2048_mid, R_mid, cIt_mid;
  vector<float> I2048_end, R_end, cIt_end;

  VectorXf P_b, P_m, P_e;
};

#endif
