#include "Scene.h"

#include "Glyphs.h"
#include "../Operation/Transforms.h"

//Constructor / Destructor
Scene::Scene(Glyphs* glyph){
  this->glyphManager = glyph;
  //---------------------------

  this->list_Mesh = new list<Mesh*>;
  this->selectNew = false;

  //---------------------------
}
Scene::~Scene(){
  delete list_Mesh;
}

//List functions
Mesh* Scene::loadCloud(string pathFile){
  bool sucess = loaderManager.load_cloud(pathFile);
  Mesh* mesh = loaderManager.get_createdMesh();
  //---------------------------

  //if well loaded
  if(sucess){
    this->is_meshNameExist(mesh);
    list_Mesh->push_back(mesh);
    this->select_specificMesh(mesh->oID);
    this->get_nameByOrder();

    //update cloud
    this->update_CloudPosition(mesh);
    this->update_allCloudData(mesh);
    this->update_dataFormat(mesh);
    this->update_oID(list_Mesh);
    this->update_COM_Initial(mesh);
  }else{
    console.AddLog("[error] Problem loading mesh");
  }

  //---------------------------
  return mesh;
}
Mesh* Scene::loadCloud(string pathFile, Matrix4f realTransformation){
  bool sucess = loaderManager.load_cloud(pathFile);
  Mesh* mesh = loaderManager.get_createdMesh();
  mat4 transfMat = eigen_to_glm_mat4(realTransformation);
  //---------------------------

  //Enter ground truth transformation matrix
  //Make transformation to initial pose to get final point positions
  mesh->transformation.RealTransformation = transfMat;
  mesh->registration.XYZ_groundTruth = mesh->location.Initial;
  Transforms transformManager;
  transformManager.make_Transformation_atomic(mesh->registration.XYZ_groundTruth, vec3(0,0,0), transfMat);

  //if well loaded
  if(sucess){
    this->is_meshNameExist(mesh);
    list_Mesh->push_back(mesh);
    this->select_specificMesh(mesh->oID);
    this->get_nameByOrder();

    //update cloud
    this->update_CloudPosition(mesh);
    this->update_allCloudData(mesh);
    this->update_dataFormat(mesh);
    this->update_oID(list_Mesh);
    this->update_COM_Initial(mesh);
  }else{
    console.AddLog("[error] Problem loading mesh");
  }

  //---------------------------
  return mesh;
}
Mesh* Scene::loadCloud(string pathFile, int lmin, int lmax){
  bool sucess = loaderManager.load_cloud_part(pathFile, lmin, lmax);
  Mesh* mesh = loaderManager.get_createdMesh();
  //---------------------------

  //if well loaded
  if(sucess){
    this->is_meshNameExist(mesh);
    list_Mesh->push_back(mesh);
    this->select_specificMesh(mesh->oID);
    this->get_nameByOrder();

    //update cloud
    this->update_CloudPosition(mesh_current);
    this->update_dataFormat(mesh);
    this->update_oID(list_Mesh);
  }else{
    console.AddLog("[error] Problem loading mesh");
  }

  //---------------------------
  return mesh;
}
Mesh* Scene::loadCloud_extracted(Mesh* mesh_in){
  bool sucess = loaderManager.load_cloud_creation(mesh_in);
  Mesh* mesh = loaderManager.get_createdMesh();
  delete mesh_in;
  //---------------------------

  //if well created
  if(sucess){
    this->is_meshNameExist(mesh);
    list_Mesh->push_back(mesh);
    this->select_specificMesh(mesh->oID);
    this->get_nameByOrder();

    //update cloud
    this->update_CloudPosition(mesh_current);
    this->update_dataFormat(mesh);
    this->update_oID(list_Mesh);
  }else{
    console.AddLog("[error] Problem loading mesh");
  }

  //---------------------------
  return mesh;
}
void Scene::saveCloud(Mesh* mesh, string pathFile){
  //---------------------------

  loaderManager.save_cloud(mesh, pathFile);

  //---------------------------
}
void Scene::saveCloud_all(string pathDir){
  //---------------------------

  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    string pathFile = pathDir + "/" + mesh->Name + ".pts";
    loaderManager.save_cloud(mesh, pathFile);
  }

  //---------------------------
}
void Scene::removeCloud(Mesh* mesh){
  //---------------------------

  if(list_Mesh->size() != 0){
    int oID = mesh->oID;
    string name =  mesh->Name;
    //---------------------------

    //Delete mesh
    this->select_specificMesh(oID);
    delete mesh_current;

    //Delete mesh iterator in list
    list<Mesh*>::iterator it = next(list_Mesh->begin(), oID);
    list_Mesh->erase(it);

    //Check for end list new selected mesh
    if(oID >= list_Mesh->size()){
      oID = 0;
    }

    this->update_oID(list_Mesh);
    this->select_specificMesh(oID);

    //---------------------------
    console.AddLog("Cloud %s removed", name.c_str());
  }
  if(list_Mesh->size() == 0){
    glyphManager->clear();
  }

  //---------------------------
}
void Scene::removeCloud_all(){
  //---------------------------

  while(list_Mesh->size() != 0){
    Mesh* mesh = *list_Mesh->begin();
    this->removeCloud(mesh);
  }

  //---------------------------
}

//Updating
void Scene::update_allCloudData(Mesh* mesh){
  //---------------------------

  if(this->is_atLeastOneMesh()){
    this->update_MinMaxCoords(mesh);
    glyphManager->update(mesh);
    mesh->NbPoints = mesh->location.OBJ.size();
  }

  //---------------------------
}
void Scene::update_CloudPosition(Mesh* mesh){
  this->update_allCloudData(mesh_current);
  //---------------------------

  //Reactualise vertex position data
  vector<vec3>& XYZ = mesh->location.OBJ;
  glBindBuffer(GL_ARRAY_BUFFER, mesh->location.VBO);
  glBufferSubData(GL_ARRAY_BUFFER, 0, XYZ.size() * sizeof(glm::vec3), &XYZ[0]);

  //---------------------------
}
void Scene::update_CloudColor(Mesh* mesh){
  //---------------------------

  //Reactualise vertex color data
  vector<vec4>& RGB = mesh->color.OBJ;
  glBindBuffer(GL_ARRAY_BUFFER, mesh->color.VBO);
  glBufferSubData(GL_ARRAY_BUFFER, 0, RGB.size() * sizeof(glm::vec4), &RGB[0]);
  mesh->intensity.heatmap == false;

  //---------------------------
}
void Scene::update_MinMaxCoords(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3 min = XYZ[0];
  vec3 max = XYZ[0];
  vec3 centroid = vec3(0, 0, 0);
  int size = XYZ.size();
  //---------------------------

  for(int i=0; i<size; i++){
    for(int j=0; j<3; j++){
      if ( XYZ[i][j] <= min[j] ) min[j] = XYZ[i][j];
      if ( XYZ[i][j] >= max[j] ) max[j] = XYZ[i][j];
      centroid[j] += XYZ[i][j];
    }
  }

  for(int j=0;j<3;j++){
    centroid[j] /= size;
  }

  //---------------------------
  mesh->location.Min = min;
  mesh->location.Max = max;
  mesh->location.COM = centroid;
}
void Scene::update_COM_Initial(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.Initial;
  vec3 centroid = vec3(0, 0, 0);
  int size = XYZ.size();
  //---------------------------

  for(int i=0; i<size; i++){
    for(int j=0; j<3; j++){
      centroid[j] += XYZ[i][j];
    }
  }

  for(int j=0;j<3;j++){
    centroid[j] /= size;
  }

  //---------------------------
  mesh->location.COM_initial = centroid;
}
void Scene::update_oID(list<Mesh*>* list){
  //---------------------------

  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);
    if(mesh->oID != i) mesh->oID = i;
  }

  //---------------------------
}
void Scene::update_dataFormat(Mesh* mesh){
  mesh->dataFormat.clear();
  string df = "XYZ";
  //---------------------------

  if(mesh->intensity.hasData) df += " - I";
  if(mesh->color.hasData) df += " - RGB";
  if(mesh->normal.hasData) df += " - Nxyz";

  //---------------------------
  mesh->dataFormat = df;
}
void Scene::update_IntensityToColor(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  //---------------------------

  for(int i=0; i<Is.size(); i++){
    RGB[i] = vec4(Is[i], Is[i], Is[i], 1.0f);
  }

  //---------------------------
  mesh->color.Buffer = RGB;
  this->update_CloudColor(mesh);
}
void Scene::update_ResetMesh(Mesh* mesh){
  //---------------------------

  //Reinitialize main data
  mesh->location.OBJ = mesh->location.Initial;
  mesh->location.Buffer = mesh->location.Initial;
  mesh->color.OBJ = mesh->color.Initial;
  mesh->color.Buffer = mesh->color.Initial;
  if(mesh->normal.hasData){
    mesh->normal.OBJ = mesh->normal.Initial;
    mesh->normal.Buffer = mesh->normal.Initial;
  }
  if(mesh->intensity.hasData){
    mesh->intensity.OBJ = mesh->intensity.Initial;
    mesh->intensity.Buffer = mesh->intensity.Initial;
    if(mesh->color.hasData == false){
      this->update_IntensityToColor(mesh);
    }
  }

  //Reset additional data
  mesh->attribut.dist.clear();
  mesh->attribut.cosIt.clear();
  mesh->attribut.It.clear();
  mesh->attribut.list_selectPoints.clear();
  mesh->attribut.list_idxPoints.clear();
  mesh->intensity.heatmap = false;
  mesh->intensity.corrected = false;
  mesh->intensity.linearized = false;
  mesh->visibility = true;
  mesh->location.root = vec3(0,0,0);
  mesh->registration.keypoints.clear();
  mesh->registration.trgpoints.clear();
  mesh->registration.XYZI.clear();
  mesh->attribut.list_idxPoints.clear();

  //Transformation matrices
  mesh->transformation.Scale = mat4(1.0);
  mesh->transformation.Translation = mat4(1.0);
  mesh->transformation.Rotation = mat4(1.0);
  mesh->transformation.TransformationMatrix = mat4(1.0);

  //Update
  this->update_allCloudData(mesh);
  this->update_CloudPosition(mesh);
  this->update_CloudColor(mesh);

  //---------------------------
}
void Scene::update_resetLocation(Mesh* mesh){
  //---------------------------

  //Reinitialize main data
  mesh->location.OBJ = mesh->location.Initial;
  mesh->location.Buffer = mesh->location.Initial;
  if(mesh->normal.hasData){
    mesh->normal.OBJ = mesh->normal.Initial;
    mesh->normal.Buffer = mesh->normal.Initial;
  }

  //Reset additional data
  mesh->attribut.dist.clear();
  mesh->attribut.cosIt.clear();
  mesh->attribut.It.clear();

  mesh->location.root = vec3(0,0,0);
  mesh->registration.keypoints.clear();
  mesh->registration.trgpoints.clear();
  mesh->registration.XYZI.clear();
  mesh->attribut.list_idxPoints.clear();

  //Transformation matrices
  mesh->transformation.Scale = mat4(1.0);
  mesh->transformation.Translation = mat4(1.0);
  mesh->transformation.Rotation = mat4(1.0);
  mesh->transformation.TransformationMatrix = mat4(1.0);

  //Update
  this->update_allCloudData(mesh);
  this->update_CloudPosition(mesh);

  //---------------------------
}

//Selection
void Scene::select_nextMesh(){
  //---------------------------

  if(list_Mesh->size() != 0){
    if(mesh_current->oID + 1 < list_Mesh->size()){
      mesh_current = *next(list_Mesh->begin(),mesh_current->oID + 1);
    }
    else{
      mesh_current = *next(list_Mesh->begin(),0);
    }
    this->update_MinMaxCoords(mesh_current);
    glyphManager->update(mesh_current);
  }else{
    glyphManager->clear();
  }

  //---------------------------
}
void Scene::select_specificMesh(int ID){
  //---------------------------

  for (int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    if(mesh->oID == ID){
      mesh_current = mesh;
      this->update_allCloudData(mesh_current);
    }
  }

  //---------------------------
}
void Scene::set_selectedMesh(Mesh* mesh){
  //---------------------------

  mesh_current = mesh;
  this->update_allCloudData(mesh_current);

  //---------------------------
}
void Scene::set_selectMeshByName(string name){
  //---------------------------

  for (int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    if(mesh->Name == name){
      mesh_current = mesh;
      this->update_allCloudData(mesh_current);
    }
  }

  //---------------------------
}
void Scene::set_MeshVisibility(Mesh* mesh, bool visibleON){
  vector<vec4>& RGBA = mesh->color.OBJ;
  //---------------------------

  //Toggle mesh visibility
  if(visibleON == true && mesh->visibility == false){
    for(int i=0; i<RGBA.size(); i++){
      RGBA[i] = vec4(RGBA[i].x, RGBA[i].y, RGBA[i].z, 1.0f);
    }
    mesh->visibility = true;
  }else if(visibleON == false && mesh->visibility == true){
    for(int i=0; i<RGBA.size(); i++){
      RGBA[i] = vec4(RGBA[i].x, RGBA[i].y, RGBA[i].z, 0.0f);
    }
    mesh->visibility = false;
  }

  //---------------------------
  this->update_CloudColor(mesh);
}

//Specific functions
vector<string> Scene::get_nameByOrder(){
  vector<string> nameByOrder;
  //---------------------------

  if(is_atLeastOneMesh()){
    string temp;

    for(int i=0; i<list_Mesh->size(); i++){
      Mesh* mesh = *next(list_Mesh->begin(), i);
      nameByOrder.push_back(mesh->Name);
    }

    bool ok = true;
    while(ok){
      ok = false;
      for(int i=0; i<list_Mesh->size()-1; i++){
        int id = strcasecmp_withNumbers(nameByOrder[i].c_str(), nameByOrder[i+1].c_str());
        if(id > 0){
          temp = nameByOrder[i];
          nameByOrder[i] = nameByOrder[i+1];
          nameByOrder[i+1] = temp;

          ok = true;
        }
      }
    }

    //Reorganize list of meshes
    list<Mesh*>* list_out = new list<Mesh*>;
    for(int i=0; i<nameByOrder.size(); i++){
      list_out->push_back(get_MeshByName(nameByOrder[i]));
    }

    list_Mesh = list_out;
  }

  //---------------------------
  return nameByOrder;
}
Mesh* Scene::get_otherMesh(){
  Mesh* mesh;
  //---------------------------

  if(list_Mesh->size() != 0){
    if(mesh_current->oID + 1 < list_Mesh->size()){
      mesh = *next(list_Mesh->begin(),mesh_current->oID + 1);
    }else{
      mesh = *next(list_Mesh->begin(),0);
    }
  }

  //---------------------------
  return mesh;
}
Mesh* Scene::get_MeshByName(string name){
  Mesh* mesh_out;
  //---------------------------

  for (int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    if(mesh->Name == name){
      mesh_out = mesh;
    }
  }

  //---------------------------
  return mesh_out;
}
Mesh* Scene::get_MeshByOID(int oID){
  Mesh* mesh_out;
  //---------------------------

  for (int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    if(mesh->oID == oID){
      mesh_out = mesh;
    }
  }

  //---------------------------
  return mesh_out;
}
int Scene::get_orderSelectedMesh(){
  vector<string> Names = get_nameByOrder();
  //---------------------------

  int order = -1;
  for(int i=0; i<Names.size(); i++){
    if(mesh_current->Name == Names[i]){
      order = i;
    }
  }

  //---------------------------
  return order;
}

bool Scene::is_MeshExist(Mesh* mesh_in){
  //---------------------------

  for (int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    if(mesh_in->ID == mesh->ID){
      return true;
    }
  }

  //---------------------------
  return false;
}
bool Scene::is_meshNameExist(Mesh* mesh_in){
  bool exist = false;
  //---------------------------

  for (int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    if(mesh->Name == mesh_in->Name){
      mesh_in->Name = mesh_in->Name + "_";
      exist = true;
    }
  }

  //---------------------------
  return exist;
}
bool Scene::is_atLeastMinNbMesh(int nbMin){
  bool result = false;
  int nb = list_Mesh->size();
  //---------------------------

  if(nb >= nbMin){
    result = true;
  }

  //---------------------------
  return result;
}
