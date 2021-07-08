#ifndef Radiometry_H
#define Radiometry_H

class Scene;
class Plotting;
class Reference;
class Ref_Operation;
class Linearization;
class Attribut;
class BundleByClass;

class RadarEquation;
class Surfacic_simplified;
class Surfacic_globalPiecewise;
class Surfacic_local;
class Surfacic_segmented;
class Separation_global;
class Separation_local;

#include "../Parameters.h"

class Radiometry
{
public:
  //Constructor / Destructor
  Radiometry(Scene* scene);
  ~Radiometry();

public:
  //Correction Methods
  void algo_RadarEquation(int num);
  void algo_surfacicSimplified(Mesh* mesh);
  void algo_separationGlobal(Mesh* mesh);
  void algo_surfacicGlobalPiecewise(Mesh* mesh);
  void algo_surfacicLocal(Mesh* mesh);
  void algo_surfacicSegmented(Mesh* mesh);

  //Correction functions
  void correction_allClouds();
  void correction_allClouds_Iini();
  void correction_allClouds_WtRefs();
  void correction_oneCloud(Mesh* mesh);
  void correction_oneCloud_Iini(Mesh* mesh);
  bool compute_RadioCorrection(Mesh* mesh);

  //Subfunctions
  void compute_IRmeans(list<Mesh*>* list);
  void compute_IsItconcat(list<Mesh*>* list);
  void compute_sortByIt(vector<float>& vec1, vector<float>& vec2);
  void compute_SpectralonAjustement();
  bool compute_list();
  bool compute_subListSpectralon();
  bool check_cloudData(Mesh* mesh);
  void set_referenceON(bool value);
  void remove_References();
  void wrt_results();

  //Plotting functions
  void plot_3Dmap();
  void plot_2Dmap();
  void plot_IbyR();
  void plot_IbyR_all();
  void plot_IbyR_Spectralon();
  void plot_IbyR_Sphere();
  void plot_IbyR_both();
  void plot_IbyCosIt(bool normalised);
  void plot_IbyCosIt_Sphere(bool normalised);
  void plot_IbyCosIt_all_Sphere();
  void plot_IbyIt_Spectralon(bool normalised);
  void plot_IbyCosIt_all();
  void plot_IbyIt_all_Sphere();
  void plot_IbyIt_allAngle_Spectralon(bool normalised);
  void plot_IbyIt_allPercentage_Spectralon_05m();
  void plot_IbyIt_allPercentage_Spectralon_10m();
  void plot_IbyIt_allPercentage_Spectralon_20m();
  void plot_IbyIt_allPercentage_Spectralon_30m();
  void plot_IbyIt_allPercentage_Spectralon_40m();
  void plot_IbyIt_Sphere(bool normalised);
  void plot_IbyIt_mesh(Mesh* mesh);
  void plot_IbyR_mesh(Mesh* mesh);
  void plot_IbyR_data(Mesh* mesh);
  void plot_nDbycosIt();
  void plot_JbycosIt();
  void plot_SpectralonAllMeans();
  void plot_ParameterSpace();
  void plot_IbyItbyR();

  //Setters / Getters
  inline int* get_algoCorrection(){ return &algoSelected;}
  inline Surfacic_globalPiecewise* get_Surfacic_globalPiecewise(){return surf_globalManager;}
  inline Surfacic_local* get_Surfacic_local(){return surf_localManager;}
  inline Surfacic_simplified* get_Surfacic_simplified(){return surf_simplManager;}
  inline Surfacic_segmented* get_Surfacic_segmented(){return surf_segmentedManager;}
  inline Separation_global* get_Separation_global(){return sepa_globalManager;}
  inline Linearization* get_Linearization(){return linManager;}
  inline Reference* get_Reference(){return refManager;}

private:
  //Classes
  Scene* sceneManager;
  Attribut* attribManager;
  Plotting* plotManager;
  BundleByClass* bundler;
  Reference* refManager;
  Ref_Operation* refopeManager;
  Linearization* linManager;

  RadarEquation* radarManager;
  Surfacic_simplified* surf_simplManager;
  Surfacic_globalPiecewise* surf_globalManager;
  Surfacic_local* surf_localManager;
  Surfacic_segmented* surf_segmentedManager;
  Separation_global* sepa_globalManager;
  Separation_local* sepa_localManager;

  //Attributs
  vector<vector<float>> I_saved, R_saved;
  vector<string> N_saved;
  vector<float> Is_std, Ic_std;
  vector<float> Is_CV, Ic_CV;
  vector<float> Im, Ic, Is;
  vector<float> R_mean, Is_mean, It_mean, std_mean;
  vector<float> Is_conc, It_conc;
  int algoSelected;
};

#endif
