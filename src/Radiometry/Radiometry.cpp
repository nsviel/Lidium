#include "Radiometry.h"
#include "Target/Reference.h"
#include "Linearization.h"
#include "Target/Ref_Operation.h"

#include "Correction/RadarEquation.h"
#include "Correction/Surfacic_simplified.h"
#include "Correction/Surfacic_globalPiecewise.h"
#include "Correction/Surfacic_local.h"
#include "Correction/Surfacic_segmented.h"
#include "Correction/Separation_global.h"
#include "Correction/Separation_local.h"

#include "../Operation/Plotting.h"
#include "../Engine/Scene.h"
#include "../Operation/Attribut.h"
#include "../Operation/Functions/BundleByClass.h"

extern struct ConsoleApp console;

//Constructor / destructor
Radiometry::Radiometry(Scene* scene){
  this->sceneManager = scene;
  //---------------------------

  //Classes
  this->attribManager = new Attribut(sceneManager);
  this->refManager = new Reference(attribManager);
  this->refopeManager = new Ref_Operation(refManager);
  this->plotManager = new Plotting();
  this->bundler = new BundleByClass();
  this->linManager = new Linearization(sceneManager, refopeManager);

  this->radarManager = new RadarEquation();
  this->surf_simplManager = new Surfacic_simplified(refManager);
  this->surf_globalManager = new Surfacic_globalPiecewise(refopeManager);
  this->surf_localManager = new Surfacic_local(refopeManager);
  this->surf_segmentedManager = new Surfacic_segmented(refopeManager);
  this->sepa_globalManager = new Separation_global(refManager);
  this->sepa_localManager = new Separation_local(refopeManager);

  //Attributs
  this->algoSelected = 0;
  plotManager->set_ticSize(15); //Default: 10

  //---------------------------
}
Radiometry::~Radiometry(){}

//Correction Methods
void Radiometry::algo_RadarEquation(int num){
  Mesh* mesh = sceneManager->get_selectedMesh();
  attribManager->compute_meshAttributs(mesh);
  //---------------------------

  switch(num){
    case 0: radarManager->compute_RadarEquation(mesh); break;
    case 1: radarManager->compute_IR2bycosIt(mesh); break;
    case 2: radarManager->compute_IbyCosIt(mesh); break;
    case 3: radarManager->compute_IbyR2(mesh); break;
  }

  //---------------------------
  mesh->intensity.OBJ = radarManager->get_Ic();
  mesh->intensity.heatmap = false;
}
void Radiometry::algo_surfacicSimplified(Mesh* mesh){
  refManager->load_SphereTarget_precomp_80d();
  //---------------------------

  //Algorithm
  if(check_cloudData(mesh) == false)
  if(surf_simplManager->algo_Bretagne(mesh)){
    Ic = surf_simplManager->get_Ic();
    Im = surf_simplManager->get_Im();
  }

  //---------------------------
}
void Radiometry::algo_surfacicGlobalPiecewise(Mesh* mesh){
  refManager->load_SphereTarget_precomp_80d();
  //---------------------------

  //Compute mesh attributs
  attribManager->compute_meshAttributs(mesh);

  //Algorithm
  if(surf_globalManager->algo(mesh)){
    Ic = surf_globalManager->get_Ic();
    Im = surf_globalManager->get_Im();
  }

  //---------------------------
}
void Radiometry::algo_surfacicLocal(Mesh* mesh){
  refManager->load_SphereTarget_precomp_80d();
  //---------------------------

  //Compute mesh attributs
  attribManager->compute_meshAttributs(mesh);

  //Check mesh data
  if(surf_localManager->algo(mesh)){
    Ic = surf_localManager->get_Ic();
    Im = surf_localManager->get_Im();
  }

  //---------------------------
}
void Radiometry::algo_surfacicSegmented(Mesh* mesh){
  refManager->load_SphereTarget_precomp_add80d();
  //---------------------------

  //Compute mesh attributs
  attribManager->compute_meshAttributs(mesh);

  //Check mesh data
  surf_segmentedManager->algo(mesh);
  Ic = surf_segmentedManager->get_Ic();
  Im = surf_segmentedManager->get_Im();


  //---------------------------
}
void Radiometry::algo_separationGlobal(Mesh* mesh){
  refManager->load_SpectralonTarget();
  //---------------------------

  //Compute mesh attributs
  attribManager->compute_meshAttributs(mesh);

  //Algorithm
  if(sepa_globalManager->algo(mesh)){
    Ic = sepa_globalManager->get_Ic();
    Im = sepa_globalManager->get_Im();
  }

  //---------------------------
}

//Correction functions
void Radiometry::correction_allClouds(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  //Correct all cloud
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    //Correction
    this->compute_RadioCorrection(mesh);

    //Update
    if(mesh->intensity.Buffer.size() != 0){
      mesh->intensity.OBJ = mesh->intensity.Buffer;
      mesh->intensity.Buffer.clear();
    }
  }

  //---------------------------
}
void Radiometry::correction_allClouds_Iini(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  //Correct all cloud
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    mesh->location.Buffer = mesh->location.OBJ;
    mesh->normal.Buffer = mesh->normal.OBJ;

    mesh->location.OBJ = mesh->location.Initial;
    mesh->normal.OBJ = mesh->normal.Initial;

    this->compute_RadioCorrection(mesh);
  }

  //Update all cloud
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    mesh->location.OBJ = mesh->location.Buffer;
    mesh->normal.OBJ = mesh->normal.Buffer;

    //For avoid heatmap (?)
    if(mesh->intensity.Buffer.size() != 0){
      mesh->intensity.OBJ = mesh->intensity.Buffer;
      mesh->intensity.Buffer.clear();
    }
  }

  //---------------------------
}
void Radiometry::correction_allClouds_WtRefs(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  Is_std.clear(); Is_CV.clear(); Ic_std.clear(); Ic_CV.clear();
  //---------------------------

  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    if(mesh->Name.find("Sphere") == std::string::npos && mesh->Name.find("Spectralon") == std::string::npos){
      this->correction_oneCloud(mesh);
    }
  }

  //---------------------------
  this->wrt_results();
}
void Radiometry::correction_oneCloud(Mesh* mesh){
  //---------------------------

  this->compute_RadioCorrection(mesh);
  mesh->intensity.OBJ = mesh->intensity.Buffer;

  //---------------------------
}
void Radiometry::correction_oneCloud_Iini(Mesh* mesh){
  mesh->location.Buffer = mesh->location.OBJ;
  mesh->normal.Buffer = mesh->normal.OBJ;
  mesh->location.OBJ = mesh->location.Initial;
  mesh->normal.OBJ = mesh->normal.Initial;
  //---------------------------

  this->compute_RadioCorrection(mesh);
  mesh->intensity.OBJ = mesh->intensity.Buffer;

  //---------------------------
  mesh->location.OBJ = mesh->location.Buffer;
  mesh->normal.OBJ = mesh->normal.Buffer;
}
bool Radiometry::compute_RadioCorrection(Mesh* mesh){
  Ic.clear(); Im.clear();
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  //Correction algorithm
  switch(algoSelected){
    case 0: this->algo_surfacicSegmented(mesh); break;
    case 1: this->algo_surfacicGlobalPiecewise(mesh); break;
    case 2: this->algo_separationGlobal(mesh); break;
    case 3: this->algo_surfacicLocal(mesh); break;
    case 4: this->algo_surfacicSimplified(mesh); break;
    case 5: this->algo_RadarEquation(0); break;
  }

  //If well functionning
  if(Ic.size() !=0 && Im.size() != 0){
    //Result
    console.AddLog("%s corrected -> Iraw: mean = %.2f, CV = %.2f | Icor: mean = %.2f, CV = %.2f",
    mesh->Name.c_str(), fct_Mean(Is), fct_CV(Is), fct_Mean(Ic), fct_CV(Ic));

    //Parameters
    mesh->intensity.heatmap = false;
    mesh->intensity.corrected = true;
    mesh->intensity.OBJ = Ic;
    mesh->intensity.Buffer = Ic;

    //Update color
    sceneManager->update_IntensityToColor(mesh);
  }else{
    console.AddLog("[error] Problem occurs during correction");
  }

  //---------------------------
  return true;
}

//Subfunctions
void Radiometry::compute_IRmeans(list<Mesh*>* list){
  attribManager->compute_meshAttributs_all();
  Is_mean.clear(); R_mean.clear(); It_mean.clear(); std_mean.clear();
  //---------------------------

  //Compute R mean and I mean for each Mesh
  for(int i=0;i<list->size();i++){
    Mesh* mesh = *next(list->begin(),i);
    vector<float>& Is = mesh->intensity.OBJ;
    vector<float>& dist = mesh->attribut.dist;
    vector<float>& It = mesh->attribut.It;

    R_mean.push_back(fct_Mean(dist));
    Is_mean.push_back(fct_Mean(Is));
    It_mean.push_back(fct_Mean(It));
    std_mean.push_back(fct_std(Is));
  }

  //---------------------------
  if(sceneManager->get_selectedMesh()->Name.find("Sphere") != std::string::npos){
    I_saved.push_back(Is_mean);
    R_saved.push_back(R_mean);
  }
}
void Radiometry::compute_IsItconcat(list<Mesh*>* list){
  attribManager->compute_meshAttributs_all();
  Is_conc.clear(); It_conc.clear();
  //---------------------------

  //Compute R mean and I mean for each Mesh
  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);
    vector<float>& Is = mesh->intensity.Initial;
    vector<float>& It = mesh->attribut.It;

    Is_conc.insert(Is_conc.end(), Is.begin(), Is.end());
    It_conc.insert(It_conc.end(), It.begin(), It.end());
  }

  vector<size_t> idx = sort_indexes(It_conc);

  vector<float> temp;
  for(int i=0; i<Is_conc.size(); i++){
    temp.push_back(Is_conc[idx[i]]);
  }

  //---------------------------
  Is_conc = temp;
}
void Radiometry::compute_sortByIt(vector<float>& vec1, vector<float>& vec2){
  //---------------------------

  vector<size_t> idx = sort_indexes(vec1);
  sort(vec1.begin(), vec1.end());
  vector<float> tempo;
  for(int i=0; i<vec2.size(); i++){
    tempo.push_back(vec2[idx[i]]);
  }

  //---------------------------
  vec2 = tempo;
}
void Radiometry::compute_SpectralonAjustement(){
  list<Mesh*>* list_99p = refManager->get_list_99p_40m_xd();
  list<Mesh*>* list_50p = refManager->get_list_50p_40m_xd();
  list<Mesh*>* list_25p = refManager->get_list_25p_40m_xd();
  list<Mesh*>* list_10p = refManager->get_list_10p_40m_xd();
  //---------------------------

  for(int i=0; i<list_99p->size() ;i++){
    Mesh* mesh_99 = *next(list_99p->begin(),i);
    Mesh* mesh_50 = *next(list_50p->begin(),i);
    Mesh* mesh_25 = *next(list_25p->begin(),i);
    Mesh* mesh_10 = *next(list_10p->begin(),i);

    vector<float>& Is_99 = mesh_99->intensity.OBJ;
    vector<float>& It_99 = mesh_99->attribut.It;
    vector<float>& Is_50 = mesh_50->intensity.OBJ;
    vector<float>& Is_25 = mesh_25->intensity.OBJ;
    vector<float>& Is_10 = mesh_10->intensity.OBJ;

    //low angle reducer
    for(int i=0; i<It_99.size(); i++){
      if(It_99[i]<5){
        Is_99[i] = Is_99[i] + 0.105;
        if(Is_99[i] >= 0.99) Is_99[i] = 0.996338;
        Is_50[i] = Is_50[i] + 0.105;
        Is_25[i] = Is_25[i] + 0.105;
        Is_10[i] = Is_10[i] + 0.105;
      }
    }
  }

  //---------------------------
}
void Radiometry::set_referenceON(bool value){
  if(refManager->is_listsCompiled() == false){
    refManager->compute_list(sceneManager->get_listMesh());
  }
  list<Mesh*>* list = refManager->get_listReference();
  //---------------------------

  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);
    sceneManager->set_MeshVisibility(mesh, value);
  }

  //---------------------------
  cout<<"Reference targets visibility: "<<value<<endl;
}

void Radiometry::remove_References(){
  if(refManager->is_listsCompiled() == false){
    refManager->compute_list(sceneManager->get_listMesh());
  }
  list<Mesh*>* list = refManager->get_listReference();
  //---------------------------

  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);
    sceneManager->removeCloud(mesh);
  }

  //---------------------------
  refManager->clear();
}
bool Radiometry::check_cloudData(Mesh* mesh){
  attribManager->compute_meshAttributs(mesh);
  //---------------------------

  if(mesh->intensity.hasData == false){
    cout<<"Correction -> No intensity data"<<endl;
    return false;
  }
  if(mesh->normal.hasData == false){
    cout<<"Correction -> No normal data"<<endl;
    return false;
  }
  if(mesh->attribut.dist.size() == 0){
    cout<<"Correction -> No distance data"<<endl;
    return false;
  }
  if(mesh->attribut.cosIt.size() == 0){
    cout<<"Correction -> No cosIt data"<<endl;
    return false;
  }
  if(mesh->intensity.corrected == true){
    cout<<"Correction -> Cloud already corrected"<<endl;
    return false;
  }
  if(mesh->location.OBJ.size() != mesh->intensity.OBJ.size()){
    cout<<"Correction -> Sizing problem"<<endl;
    return false;
  }

  //---------------------------
  return true;
}
