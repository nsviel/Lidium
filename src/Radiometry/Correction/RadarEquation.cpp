#include "RadarEquation.h"

#include "../../Operation/Plotting.h"

//Constructor / Destructor
RadarEquation::RadarEquation(){
  //---------------------------

  plotManager = new Plotting();

  this->D = (3.8/1000); //FARO Focus 3D
  this->lambda = (905/1000000000); //FARO Focus 3D
  this->Pe = 0.02; //FARO Focus 3D

  //---------------------------
}
RadarEquation::~RadarEquation(){}

void RadarEquation::compute_RadarEquation(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  vector<float>& cosIt = mesh->attribut.cosIt;
  int size = Is.size();
  Ic.clear();
  //---------------------------

  //Make intensity correction according to radar's equation
  for(int i=0; i<size; i++){
    float Icorrect = Is[i] * (4 / (Pe * pow(D,2))) * (pow(dist[i],2) / pow(R_ref,2)) * (1 / pow(cosIt[i],2));
    Ic.push_back(Icorrect);
  }

  //---------------------------
  this->plot_intensityCorrected(mesh);
}
void RadarEquation::compute_IR2bycosIt(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  vector<float>& cosIt = mesh->attribut.cosIt;
  int size = Is.size();
  Ic.clear();
  //---------------------------

  //Make intensity correction according to radar's equation
  for(int i=0; i<size; i++){
    float Icorrect = Is[i] * (pow(dist[i],2) / pow(R_ref,2)) * (1 / pow(cosIt[i],2));
    Ic.push_back(Icorrect);
  }

  //---------------------------
  this->plot_intensityCorrected(mesh);
}
void RadarEquation::compute_IbyR2(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  float R_ref = 6.0f;
  int size = Is.size();
  Ic.clear();
  //---------------------------

  // Ic = Is * (R / Rs)
  for(int i=0; i<size; i++){
    float Icorr = Is[i] * (pow(dist[i],2) / pow(R_ref,2));
    Ic.push_back(Icorr);
  }

  //---------------------------
  this->plot_intensityCorrected(mesh);
}
void RadarEquation::compute_IbyCosIt(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  vector<float>& cosIt = mesh->attribut.cosIt;
  int size = Is.size();
  Ic.clear();
  //---------------------------

  // Ic = Is / cos(It)
  for(int i=0; i<size; i++){
    float Icorr = Is[i] * (1 / pow(cosIt[i],2));
    Ic.push_back(Icorr);
  }

  //---------------------------
  this->plot_intensityCorrected(mesh);
}

//Plotting functions
void RadarEquation::plot_intensityCorrected(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  plotManager->plot_2histograms(Is, Ic);

  //---------------------------
}
