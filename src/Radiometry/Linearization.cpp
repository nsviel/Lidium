#include "Linearization.h"

#include "Target/Ref_Operation.h"
#include "Target/Reference.h"
#include "../Engine/Scene.h"
#include "../Operation/Attribut.h"
#include "../Operation/Optimization/SpeudoInverse.h"

extern struct ConsoleApp console;

//Constructor / Destructor
Linearization::Linearization(Scene* scene, Ref_Operation* ope){
  this->sceneManager = scene;
  this->opeManager = ope;
  //---------------------------

  this->refManager = opeManager->get_refManager();
  this->attribManager = new Attribut(sceneManager);

  this->mode = 0;
  this->gamma_G = -1;
  this->gamma_D = -1;

  //---------------------------
}
Linearization::~Linearization(){}

//General functions
void Linearization::algo_linearization(Mesh* mesh, int method){
  //---------------------------

  switch(method){
    case 0:{//Surfacic global
      this->lin_SurfacicGlobal(mesh);
      break;
    }
    case 1:{//Separation global
      this->lin_SeparationGlobal(mesh);
      break;
    }
    case 2:{//Surfacic local
      this->lin_SurfacicLocal(mesh);
      break;
    }
  }

  //---------------------------
  sceneManager->update_CloudColor(mesh);
  console.AddLog("%s linearized", mesh->Name.c_str());
}
void Linearization::algo_reverse(Mesh* mesh, int method){
  //---------------------------

  switch(method){
    case 0:{//Surfacic global
      this->rev_SurfacicGlobal(mesh);
      break;
    }
    case 1:{//Separation global
      this->rev_SeparationGlobal(mesh);
      break;
    }
  }

  //---------------------------
  sceneManager->update_CloudColor(mesh);
  console.AddLog("%s linearization reversed", mesh->Name.c_str());
}

//Linearization
void Linearization::lin_SurfacicGlobal(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  //Compute coefficients if necessary
  if(gamma_G == -1 && gamma_D == -1){
    this->compute_gammaCoefficients();
  }

  //Apply linearization
  for(int i=0; i<Is.size(); i++){
    Is[i] = exp(log(Is[i])/gamma_G - log(gamma_D));

    //Check limits
    if(Is[i] > 1){
      Is[i] = 1;
    }else if(Is[i] < 0){
      Is[i] = 0;
    }
  }

  //---------------------------
  mesh->intensity.linearized = true;
  sceneManager->update_IntensityToColor(mesh);
}
void Linearization::lin_SeparationGlobal(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  int size = Is.size();
  float D = 0.24;
  float E = 5.23;
  //---------------------------

  for(int i=0; i<size; i++){
    Is[i] = D*pow((exp(Is[i]) - 1), E);

    //Contrainte
    if(Is[i] > 1){
      Is[i] = 1;
    }else if(Is[i] < 0){
      Is[i] = 0;
    }
  }

  //---------------------------
  mesh->intensity.linearized = true;
  sceneManager->update_IntensityToColor(mesh);
}
void Linearization::lin_SurfacicLocal(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  int size = Is.size();
  float D = 0.1513;
  float E = 5.1416;
  //---------------------------

  for(int i=0; i<size; i++){
    Is[i] = D*pow((exp(Is[i]) - 1), E);

    //Contrainte
    if(Is[i] > 1){
      Is[i] = 1;
    }else if(Is[i] < 0){
      Is[i] = 0;
    }
  }

  //---------------------------
  mesh->intensity.linearized = true;
  sceneManager->update_IntensityToColor(mesh);
}

//Reverse
void Linearization::rev_SurfacicGlobal(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  int size = Is.size();
  float A = 1.46;
  float B = 0.21;
  //---------------------------

  for(int i=0; i<size; i++){
    Is[i] = log(1 + A * pow(Is[i],B));
  }

  //---------------------------
  sceneManager->update_IntensityToColor(mesh);
}
void Linearization::rev_SeparationGlobal(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  int size = Is.size();
  float A = 1.30;
  float B = 0.20;
  //---------------------------

  for(int i=0; i<size; i++){
    Is[i] = log(1 + A * pow(Is[i],B));
  }

  //---------------------------
  sceneManager->update_IntensityToColor(mesh);
}

//Math functions
void Linearization::compute_SpectralonValues(){
  /* WORK IN PROGRESS */
  list<Mesh*>* list_10p_xm_xd = refManager->get_list_10p_xm_xd();
  list<Mesh*>* list_25p_xm_xd = refManager->get_list_25p_xm_xd();
  list<Mesh*>* list_50p_xm_xd = refManager->get_list_50p_xm_xd();
  list<Mesh*>* list_99p_xm_xd = refManager->get_list_99p_xm_xd();
  //---------------------------

  for(int i=0; i<list_10p_xm_xd->size(); i++){
    Mesh* mesh = *next(list_10p_xm_xd->begin(),i);
  }

  //---------------------------
}
void Linearization::compute_gammaCoefficients(){
  SpeudoInverse invManager;
  VectorXf I_mes(4), I_cal(4), A(4), B(4);
  VectorXf P = VectorXf::Zero(2);
  //---------------------------

  I_mes << 0.892,	0.832, 0.757, 0.635;
  I_cal << 0.99, 0.57, 0.28, 0.11;

  A << log(I_mes(0)), log(I_mes(1)), log(I_mes(2)), log(I_mes(3));
  B << log(I_cal(0)), log(I_cal(1)), log(I_cal(2)), log(I_cal(3));

  MatrixXf J(4,2);
  for(int j=0; j<A.size(); j++){
    J(j, 0) = 1;
    J(j, 1) = B(j);
  }

  MatrixXf JINV = invManager.SpeudoInverse_orthoDecomp(J);

  VectorXf Z(4), E(4);
  for(int i=0; i<10; i++){
    for(int j=0; j<A.size(); j++){
      Z(j) = P(0) + P(1) * B(j);
      E(j) = Z(j) - A(j);
    }

    P = P - JINV * E;
  }

  gamma_C = P(0);
  gamma_G = P(1);
  gamma_D = exp(gamma_C / gamma_G);

  //For result checking
  VectorXf I_log(4);
  VectorXf I_lin(4);
  for(int i=0; i<4; i++){
    I_log(i) = pow((gamma_D * I_cal(i)), gamma_G);
    I_lin(i) = exp(log(I_mes(i)) / gamma_G - log(gamma_D));
  }

  //---------------------------
}
