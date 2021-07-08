#include "Reference.h"

#include "../../Operation/Attribut.h"
#include "../../Load/Loader.h"

//Constructor / Destructor
Reference::Reference(Attribut* attrib){
  this->attribManager = attrib;
  //---------------------------

  this->loaderManager = new Loader();

  this->list_Sphere = new list<Mesh*>;
  this->list_Spectralon = new list<Mesh*>;

  this->list_10p_xm_xd = new list<Mesh*>;
  this->list_25p_xm_xd = new list<Mesh*>;
  this->list_50p_xm_xd = new list<Mesh*>;
  this->list_99p_xm_xd = new list<Mesh*>;

  this->list_xp_10m_xd = new list<Mesh*>;
  this->list_xp_20m_xd = new list<Mesh*>;
  this->list_xp_30m_xd = new list<Mesh*>;
  this->list_xp_40m_xd = new list<Mesh*>;

  this->list_10p_05m_xd = new list<Mesh*>;
  this->list_25p_05m_xd = new list<Mesh*>;
  this->list_50p_05m_xd = new list<Mesh*>;
  this->list_99p_05m_xd = new list<Mesh*>;

  this->list_10p_10m_xd = new list<Mesh*>;
  this->list_25p_10m_xd = new list<Mesh*>;
  this->list_50p_10m_xd = new list<Mesh*>;
  this->list_99p_10m_xd = new list<Mesh*>;

  this->list_10p_20m_xd = new list<Mesh*>;
  this->list_25p_20m_xd = new list<Mesh*>;
  this->list_50p_20m_xd = new list<Mesh*>;
  this->list_99p_20m_xd = new list<Mesh*>;

  this->list_10p_30m_xd = new list<Mesh*>;
  this->list_25p_30m_xd = new list<Mesh*>;
  this->list_50p_30m_xd = new list<Mesh*>;
  this->list_99p_30m_xd = new list<Mesh*>;

  this->list_10p_40m_xd = new list<Mesh*>;
  this->list_25p_40m_xd = new list<Mesh*>;
  this->list_50p_40m_xd = new list<Mesh*>;
  this->list_99p_40m_xd = new list<Mesh*>;

  this->list_10p_xm = new list<Mesh*>;
  this->list_25p_xm = new list<Mesh*>;
  this->list_50p_xm = new list<Mesh*>;
  this->list_99p_xm = new list<Mesh*>;

  this->list_Spec_dist = new list<Mesh*>;
  this->list_Spec_angle = new list<Mesh*>;

  this->list_Reference = new list<Mesh*>;
  this->list_Ref_dist = new list<Mesh*>;
  this->list_Ref_angle = new list<Mesh*>;

  this->listsCompiled = false;
  this->ref_Spectralon = false;
  this->ref_Sphere = false;

  this->path_calibrationTargets = "../media/calibrationTargets";
  this->path_sphereTarget = path_calibrationTargets + "/Sphere";
  this->path_sphereTarget_80d = path_calibrationTargets + "/Sphere_80d";
  this->path_sphereTarget_add80d = path_calibrationTargets + "/Sphere_additional_80d";
  this->path_spectralonTarget_dist = path_calibrationTargets + "/Spectralon_distance";
  this->path_spectralonTarget_angle = path_calibrationTargets + "/Spectralon_angle/10m";

  //---------------------------
}
Reference::~Reference(){}

//Load functions
bool Reference::load_SphereTarget_precomp(){
  list<Mesh*>* list_ref = new list<Mesh*>;
  if(isref_Sphere()) return false;
  //---------------------------

  //Load sphere targets
  this->load_calibrationTargets(list_ref, path_sphereTarget);

  //Inserte into sphere list
  for(int i=0; i<list_ref->size() ;i++){
    Mesh* mesh = *next(list_ref->begin(),i);

    attribManager->compute_meshAttributs(mesh);
    list_Sphere->push_back(mesh);
    list_Reference->push_back(mesh);
    ref_Sphere = true;
  }

  //---------------------------
  return true;
}
bool Reference::load_SphereTarget_precomp_80d(){
  list<Mesh*>* list_ref = new list<Mesh*>;
  if(isref_Sphere()) return false;
  //---------------------------

  //Load sphere targets
  this->load_calibrationTargets(list_ref, path_sphereTarget_80d);

  //Insert into sphere list
  for(int i=0; i<list_ref->size() ;i++){
    Mesh* mesh = *next(list_ref->begin(),i);

    attribManager->compute_meshAttributs(mesh);
    list_Sphere->push_back(mesh);
    list_Reference->push_back(mesh);
    ref_Sphere = true;
  }

  //---------------------------
  return true;
}
bool Reference::load_SphereTarget_precomp_add80d(){
  list<Mesh*>* list_ref = new list<Mesh*>;
  if(isref_Sphere()) return false;
  //---------------------------

  //Load sphere targets
  bool sucess = load_calibrationTargets(list_ref, path_sphereTarget_add80d);
  if(sucess == false){
    cout<<"Problem loading list of reference targets"<<endl;
  }

  //Inserte into sphere list
  for(int i=0; i<list_ref->size() ;i++){
    Mesh* mesh = *next(list_ref->begin(),i);

    attribManager->compute_meshAttributs(mesh);
    list_Sphere->push_back(mesh);
    list_Reference->push_back(mesh);
    ref_Sphere = true;
  }

  //---------------------------
  return true;
}
bool Reference::load_SpectralonTarget(){
  char path[PATH_MAX];
  list<Mesh*>* list_ref = new list<Mesh*>;
  //---------------------------

  this->load_calibrationTargets(list_ref, path_spectralonTarget_dist);
  this->load_calibrationTargets(list_ref, path_spectralonTarget_angle);
  this->compute_list(list_ref);

  //---------------------------
  return true;
}

bool Reference::load_calibrationTargets(list<Mesh*>* list, string path_str){
  char path[PATH_MAX];
  vector<string> list_path;
  //---------------------------

  if(realpath(path_str.c_str(), path)){
    //Get list of file in folder
    for(const auto& entry : std::experimental::filesystem::directory_iterator(path)){
      string name = entry.path();
      list_path.push_back(name);
    }

    //Sort the list
    sort(list_path.begin(), list_path.end());

    //Load all files
    for(int i=0; i<list_path.size(); i++){
      loaderManager->load_cloud_silent(list_path[i]);
      Mesh* mesh = loaderManager->get_createdMesh();
      list->push_back(mesh);
    }
  }

  //---------------------------
  if(list->size() == 0){
    return false;
  }
  return true;
}
bool Reference::load_References_path(string path){
  list<Mesh*>* list_ref = new list<Mesh*>;
  //---------------------------

  this->load_calibrationTargets(list_ref, path);
  this->compute_list(list_ref);

  //---------------------------
  return true;
}

//Reference lists
bool Reference::clear(){
  //---------------------------

  this->listsCompiled = false;
  this->ref_Spectralon = false;
  this->ref_Sphere = false;

  list_Sphere->clear();
  list_Spectralon->clear();
  list_Reference->clear();

  list_Ref_dist->clear();
  list_Ref_angle->clear();

  list_10p_xm_xd->clear();
  list_25p_xm_xd->clear();
  list_50p_xm_xd->clear();
  list_99p_xm_xd->clear();
  list_xp_10m_xd->clear();
  list_xp_20m_xd->clear();
  list_xp_30m_xd->clear();
  list_xp_40m_xd->clear();

  list_10p_05m_xd->clear();
  list_25p_05m_xd->clear();
  list_50p_05m_xd->clear();
  list_99p_05m_xd->clear();
  list_10p_10m_xd->clear();
  list_25p_10m_xd->clear();
  list_50p_10m_xd->clear();
  list_99p_10m_xd->clear();
  list_10p_20m_xd->clear();
  list_25p_20m_xd->clear();
  list_50p_20m_xd->clear();
  list_99p_20m_xd->clear();
  list_10p_30m_xd->clear();
  list_25p_30m_xd->clear();
  list_50p_30m_xd->clear();
  list_99p_30m_xd->clear();
  list_10p_40m_xd->clear();
  list_25p_40m_xd->clear();
  list_50p_40m_xd->clear();
  list_99p_40m_xd->clear();

  list_10p_xm->clear();
  list_25p_xm->clear();
  list_50p_xm->clear();
  list_99p_xm->clear();

  list_Spec_angle->clear();
  list_Spec_dist->clear();

  //---------------------------
  return true;
}
bool Reference::compute_list(list<Mesh*>* list){
  bool sucess = true;
  this->clear();
  //---------------------------

  //General lists
  for(int i=0; i<list->size() ;i++){
    Mesh* mesh = *next(list->begin(),i);

    if(mesh->Name.find("Sphere") != std::string::npos){
      attribManager->compute_meshAttributs(mesh);
      list_Sphere->push_back(mesh);
      list_Reference->push_back(mesh);
      ref_Sphere = true;
    }
    if(mesh->Name.find("Spectralon") != std::string::npos){
      attribManager->compute_meshAttributs(mesh);
      list_Spectralon->push_back(mesh);
      list_Reference->push_back(mesh);
      ref_Spectralon = true;
    }
  }

  //Specialized lists
  if(ref_Sphere){
    sucess = this->extract_listSphere();
  }
  if(ref_Spectralon){
    sucess = this->extract_listSpectralon();
  }

  //---------------------------
  if(list_Reference->size() != 0 && sucess){
    listsCompiled = true;
    return true;
  }else{
    return false;
  }
}
bool Reference::extract_listSphere(){
  //---------------------------

  for(int i=0;i<list_Sphere->size();i++){
    Mesh* mesh = *next(list_Sphere->begin(),i);

    list_Ref_dist->push_back(mesh);
    list_Ref_angle->push_back(mesh);
  }

  //--------------------
  return true;
}
bool Reference::extract_listSpectralon(){
  //---------------------------

  for(int i=0;i<list_Spectralon->size();i++){
    Mesh* mesh = *next(list_Spectralon->begin(),i);

    //xp
    if(mesh->Name.find("10p") != std::string::npos){
      list_10p_xm_xd->push_back(mesh);
    }
    if(mesh->Name.find("25p") != std::string::npos){
      list_25p_xm_xd->push_back(mesh);
    }
    if(mesh->Name.find("50p") != std::string::npos){
      list_50p_xm_xd->push_back(mesh);
    }
    if(mesh->Name.find("99p") != std::string::npos){
      list_99p_xm_xd->push_back(mesh);
    }

    //Angles
    //xd by p 05m + xd xm
    if(mesh->Name.find("d") != std::string::npos &&
      mesh->Name.find("_5m") != std::string::npos){
      if(mesh->Name.find("10p") != std::string::npos){
        list_10p_05m_xd->push_back(mesh);
      }
      if(mesh->Name.find("25p") != std::string::npos){
        list_25p_05m_xd->push_back(mesh);
      }
      if(mesh->Name.find("50p") != std::string::npos){
        list_50p_05m_xd->push_back(mesh);
      }
      if(mesh->Name.find("99p") != std::string::npos){
        list_99p_05m_xd->push_back(mesh);
      }
    }
    //xd by p 10m + xd xm
    if(mesh->Name.find("0d") != std::string::npos &&
      mesh->Name.find("10m") != std::string::npos){
      if(mesh->Name.find("10p") != std::string::npos){
        list_10p_10m_xd->push_back(mesh);

      }
      if(mesh->Name.find("25p") != std::string::npos){
        list_25p_10m_xd->push_back(mesh);
      }
      if(mesh->Name.find("50p") != std::string::npos){
        list_50p_10m_xd->push_back(mesh);
      }
      if(mesh->Name.find("99p") != std::string::npos){
        list_99p_10m_xd->push_back(mesh);
      }
    }

    //xd by p 20m + xd xm
    if(mesh->Name.find("0d") != std::string::npos &&
      mesh->Name.find("20m") != std::string::npos){
      if(mesh->Name.find("10p") != std::string::npos){
        list_10p_20m_xd->push_back(mesh);
      }
      if(mesh->Name.find("25p") != std::string::npos){
        list_25p_20m_xd->push_back(mesh);
      }
      if(mesh->Name.find("50p") != std::string::npos){
        list_50p_20m_xd->push_back(mesh);
      }
      if(mesh->Name.find("99p") != std::string::npos){
        list_99p_20m_xd->push_back(mesh);
      }
    }
    //xd by p 30m + xd xm
    if(mesh->Name.find("0d") != std::string::npos &&
      mesh->Name.find("30m") != std::string::npos){
      if(mesh->Name.find("10p") != std::string::npos){
        list_10p_30m_xd->push_back(mesh);
      }
      if(mesh->Name.find("25p") != std::string::npos){
        list_25p_30m_xd->push_back(mesh);
      }
      if(mesh->Name.find("50p") != std::string::npos){
        list_50p_30m_xd->push_back(mesh);
      }
      if(mesh->Name.find("99p") != std::string::npos){
        list_99p_30m_xd->push_back(mesh);
      }
    }
    //xd by p 40m + xd xm
    if(mesh->Name.find("0d") != std::string::npos &&
      mesh->Name.find("40m") != std::string::npos){
      if(mesh->Name.find("10p") != std::string::npos){
        list_10p_40m_xd->push_back(mesh);
      }
      if(mesh->Name.find("25p") != std::string::npos){
        list_25p_40m_xd->push_back(mesh);
      }
      if(mesh->Name.find("50p") != std::string::npos){
        list_50p_40m_xd->push_back(mesh);
      }
      if(mesh->Name.find("99p") != std::string::npos){
        list_99p_40m_xd->push_back(mesh);
      }
    }

    if(mesh->Name.find("0d") != std::string::npos){
      //List Spectralon angle by distance
      if(mesh->Name.find("10m") != std::string::npos){
        list_xp_10m_xd->push_back(mesh);
      }
      if(mesh->Name.find("20m") != std::string::npos){
        list_xp_20m_xd->push_back(mesh);
      }
      if(mesh->Name.find("30m") != std::string::npos){
        list_xp_30m_xd->push_back(mesh);
      }
      if(mesh->Name.find("40m") != std::string::npos){
        list_xp_40m_xd->push_back(mesh);
      }
    }

    //all m by p
    if(mesh->Name.find("d") == std::string::npos){
      if(mesh->Name.find("10p") != std::string::npos){
        list_10p_xm->push_back(mesh);
      }
      if(mesh->Name.find("25p") != std::string::npos){
        list_25p_xm->push_back(mesh);
      }
      if(mesh->Name.find("50p") != std::string::npos){
        list_50p_xm->push_back(mesh);
      }
      if(mesh->Name.find("99p") != std::string::npos){
        list_99p_xm->push_back(mesh);
      }
    }

    //all d 99p 10m
    if(mesh->Name.find("0d") != std::string::npos &&
      mesh->Name.find("10m") != std::string::npos &&
      mesh->Name.find("99p") != std::string::npos){
      list_Ref_angle->push_back(mesh);
    }
    //all d
    if(mesh->Name.find("0d") != std::string::npos){
      list_Spec_angle->push_back(mesh);
    }
    //all d
    if(mesh->Name.find("0d") == std::string::npos){
      list_Spec_dist->push_back(mesh);
    }
    //99p xm
    if(mesh->Name.find("0d") == std::string::npos &&
      mesh->Name.find("99p") != std::string::npos){
      list_Ref_dist->push_back(mesh);
    }
  }

  //--------------------
  return true;
}

//Subfunctions
bool Reference::is_MeshReference(Mesh* mesh){
  //---------------------------

  for(int i=0; i<list_Sphere->size() ;i++){
    Mesh* mesh_list = *next(list_Sphere->begin(),i);
    if(mesh->Name == mesh_list->Name){
      return true;
    }
  }

  for(int i=0; i<list_Spectralon->size() ;i++){
    Mesh* mesh_list = *next(list_Spectralon->begin(),i);
    if(mesh->Name == mesh_list->Name){
      return true;
    }
  }

  //---------------------------
  return false;
}

Mesh* Reference::get_specificSpectralon(string p, string m, bool degree){
  Mesh* mesh_out;
  //---------------------------

  for(int i=0;i<list_Spectralon->size();i++){
    Mesh* mesh = *next(list_Spectralon->begin(),i);

    if(mesh->Name.find(p) == std::string::npos &&
      mesh->Name.find(m) == std::string::npos){

      if(degree){
        if(mesh->Name.find("0d") != std::string::npos){
          mesh_out = mesh;
        }
      }else{
        if(mesh->Name.find("0d") == std::string::npos){
          mesh_out = mesh;
        }
      }
    }
  }

  //---------------------------
  return mesh_out;
}
Mesh* Reference::get_specificSphere(string m){
  Mesh* mesh_out;
  //---------------------------

  for(int i=0;i<list_Sphere->size();i++){
    Mesh* mesh = *next(list_Sphere->begin(),i);

    if(mesh->Name.find(m) != std::string::npos){
      mesh_out = mesh;
    }
  }

  //---------------------------
  return mesh_out;
}
