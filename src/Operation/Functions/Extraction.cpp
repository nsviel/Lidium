#include "Extraction.h"

#include "../Attribut.h"
#include "../../Engine/Scene.h"

//Constructor / destructor
Extraction::Extraction(Scene* scene){
  this->sceneManager = scene;
  //---------------------------

  this->attribManager = new Attribut(sceneManager);

  this->list_part = new list<subpart*>;
  this->highlightON = false;
  this->sliceON = false;
  this->ID_cloud = 0;
  this->ID_part = 0;

  //---------------------------
}
Extraction::~Extraction(){}

//Extraction
void Extraction::fct_extractCloud(Mesh* mesh){
  Mesh* mesh_out = new Mesh();
  vector<vec3>& XYZ = mesh->location.OBJ;
  const vector<vec4>& RGB = mesh->color.Initial;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  vec3& max = mesh->location.Max;
  vec3& min = mesh->location.Min;
  mesh_out->Format = ".pts";
  mesh_out->Name = mesh->Name + "_" + "part" + to_string(ID_cloud);
  ID_cloud++;
  //---------------------------

  //Take values between sliceMin and sliceMax
  for (int i=0; i<XYZ.size(); i++)
    if(XYZ[i].x >= min.x && XYZ[i].x <= max.x &&
      XYZ[i].y >= min.y && XYZ[i].y <= max.y &&
      XYZ[i].z >= min.z && XYZ[i].z <= max.z){
      //Location
      if(sliceON){
        mesh_out->location.OBJ.push_back(vec3(XYZ[i].x, XYZ[i].y,  min.z));
      }
      else{
        mesh_out->location.OBJ.push_back(vec3(XYZ[i].x, XYZ[i].y,  XYZ[i].z));
      }
      //Color
      if(mesh->color.hasData){
        mesh_out->color.OBJ.push_back(vec4(RGB[i].x, RGB[i].y, RGB[i].z, RGB[i].w));
        mesh_out->color.hasData = true;
      }
      //Normal
      if(mesh->normal.hasData){
        mesh_out->normal.OBJ.push_back(vec3(Nxyz[i].x, Nxyz[i].y, Nxyz[i].z));
        mesh_out->normal.hasData = true;
      }
      //Intensity
      if(mesh->intensity.hasData){
        mesh_out->intensity.OBJ.push_back(Is[i]);
        mesh_out->intensity.hasData = true;
      }
    }

  //---------------------------
  if(mesh_out->location.OBJ.size() != 0){
    //Create slice if any points
    sceneManager->loadCloud_extracted(mesh_out);
  }
  else{
    cout<<"No points detected"<<endl;
  }
}
void Extraction::fct_extractSelected(Mesh* mesh){
  Mesh* mesh_out = new Mesh();
  vector<vec3>& XYZ = mesh->location.OBJ;
  const vector<vec4>& RGB = mesh->color.Initial;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;

  mesh_out->Format = ".pts";
  mesh_out->Name = mesh->Name + "_" + "part" + to_string(ID_cloud);
  ID_cloud++;
  vector<int>& idx = mesh->attribut.list_selectPoints;
  //---------------------------

  for(int i=0; i<idx.size(); i++){
    mesh_out->location.OBJ.push_back(vec3(XYZ[idx[i]].x, XYZ[idx[i]].y,  XYZ[idx[i]].z));

    //Color
    if(mesh->color.hasData){
      mesh_out->color.OBJ.push_back(vec4(RGB[idx[i]].x, RGB[idx[i]].y, RGB[idx[i]].z, RGB[idx[i]].w));
      mesh_out->color.hasData = true;
    }

    //Normal
    if(mesh->normal.hasData){
      mesh_out->normal.OBJ.push_back(vec3(Nxyz[idx[i]].x, Nxyz[idx[i]].y, Nxyz[idx[i]].z));
      mesh_out->normal.hasData = true;
    }

    //Intensity
    if(mesh->intensity.hasData){
      mesh_out->intensity.OBJ.push_back(Is[idx[i]]);
      mesh_out->intensity.hasData = true;
    }
  }

  //---------------------------
  idx.clear();
  if(mesh_out->location.OBJ.size() != 0){
    sceneManager->loadCloud_extracted(mesh_out);
  }
  else{
    cout<<"No points selected"<<endl;
  }
}
void Extraction::fct_cutCloud(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3& max = mesh->location.Max;
  vec3& min = mesh->location.Min;
  vector<int> idx;
  //---------------------------

  //Take values between sliceMin and sliceMax
  for(int i=0; i<XYZ.size(); i++){
    if(XYZ[i].x < min.x || XYZ[i].x > max.x ||
      XYZ[i].y < min.y || XYZ[i].y > max.y ||
      XYZ[i].z < min.z || XYZ[i].z > max.z){
      idx.push_back(i);
    }
  }

  //Supress non selected points
  attribManager->make_supressPoints(mesh, idx);

  //Reorder data
  mesh->color.Buffer =mesh->color.OBJ;

  //---------------------------
  sceneManager->update_allCloudData(mesh);
}
void Extraction::fct_cutCloud_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  Mesh* mesh_selected = sceneManager->get_selectedMesh();
  vec3& max = mesh_selected->location.Max;
  vec3& min = mesh_selected->location.Min;
  //---------------------------

  for(int i=0;i<list_Mesh->size();i++){
    //Select ieme Point Cloud
    Mesh* mesh = *next(list_Mesh->begin(),i);

    vector<vec3>& XYZ = mesh->location.OBJ;
    vector<int> idx;

    //Take values between sliceMin and sliceMax
    for (int i=0; i<XYZ.size(); i++){
      if(XYZ[i].x < min.x || XYZ[i].x > max.x ||
        XYZ[i].y < min.y || XYZ[i].y > max.y ||
        XYZ[i].z < min.z || XYZ[i].z > max.z){
        idx.push_back(i);
      }
    }

    //Supress non selected points
    attribManager->make_supressPoints(mesh, idx);

    //Reorder data
    mesh->color.Buffer =mesh->color.OBJ;
  }

  //---------------------------
}
void Extraction::fct_highlighting(Mesh* mesh){
  vec3 max = mesh->location.Max;
  vec3 min = mesh->location.Min;
  vector<vec3>& pos = mesh->location.OBJ;
  vector<vec4>& color = mesh->color.OBJ;
  vector<vec4>& RGB = mesh->color.Buffer;
  //---------------------------

  if(highlightON == true){
    for(int i=0; i<pos.size(); i++){
      if(pos[i].x >= min.x &&
        pos[i].y >= min.y &&
        pos[i].z >= min.z &&
        pos[i].x <= max.x &&
        pos[i].y <= max.y &&
        pos[i].z <= max.z){
        //Qualify color according to previous unlighting color
        color[i] = vec4(1,color[i].y,color[i].z,1);
      }
      else{
        //Restaure original color
        color[i] = RGB[i];
      }
    }
  }
  else{
    mesh->color.OBJ = RGB;
  }

  //---------------------------
  sceneManager->update_CloudColor(mesh);
}
void Extraction::fct_merging_list(vector<Mesh*> list_part){
  Mesh* part_1;
  Mesh* part_2;
  Mesh* mesh_out = new Mesh();
  //---------------------------

  for(int i=0; i<list_part.size()-1; i++){
    part_1 = list_part[i];
    part_2 = list_part[i+1];

    //Location
    vector<vec3>& XYZ_1 = part_1->location.OBJ;
    vector<vec3>& XYZ_2 = part_2->location.OBJ;
    vector<vec3>& XYZ_out = mesh_out->location.OBJ;

    XYZ_out.insert( XYZ_out.end(), XYZ_1.begin(), XYZ_1.end());
    XYZ_out.insert( XYZ_out.end(), XYZ_2.begin(), XYZ_2.end());

    //Color
    if(part_1->color.hasData && part_2->color.hasData){
      vector<vec4>& RGB_1 = part_1->color.OBJ;
      vector<vec4>& RGB_2 = part_2->color.OBJ;
      vector<vec4>& RGB_out = mesh_out->color.OBJ;

      RGB_out.insert( RGB_out.end(), RGB_1.begin(), RGB_1.end());
      RGB_out.insert( RGB_out.end(), RGB_2.begin(), RGB_2.end());

      mesh_out->color.hasData = true;
    }
    //Normal
    if(part_1->normal.hasData && part_2->normal.hasData){
      vector<vec3>& Nxyz_1 = part_1->normal.OBJ;
      vector<vec3>& Nxyz_2 = part_2->normal.OBJ;
      vector<vec3>& Nxyz_out = mesh_out->normal.OBJ;

      Nxyz_out.insert( Nxyz_out.end(), Nxyz_1.begin(), Nxyz_1.end());
      Nxyz_out.insert( Nxyz_out.end(), Nxyz_2.begin(), Nxyz_2.end());

      mesh_out->normal.hasData = true;
    }
    //Intensity
    if(part_1->intensity.hasData && part_2->intensity.hasData){
      vector<float>& Is_1 = part_1->intensity.OBJ;
      vector<float>& Is_2 = part_2->intensity.OBJ;
      vector<float>& Is_out = mesh_out->intensity.OBJ;

      Is_out.insert( Is_out.end(), Is_1.begin(), Is_1.end());
      Is_out.insert( Is_out.end(), Is_2.begin(), Is_2.end());

      mesh_out->intensity.hasData = true;
    }
  }

  //Infos
  mesh_out->Name = part_1->Name;
  mesh_out->Format = ".pts";
  mesh_out->NbPoints = mesh_out->location.OBJ.size();

  //---------------------------
  if(mesh_out->NbPoints > 0){
    sceneManager->loadCloud_extracted(mesh_out);
  }
  else{
    cout<<"No points available"<<endl;
  }
}
void Extraction::fct_merging_newCloud(Mesh* mesh_1, Mesh* mesh_2){
  Mesh* mesh_out = new Mesh();
  //---------------------------

  //Location
  vector<vec3>& XYZ_1 = mesh_1->location.OBJ;
  vector<vec3>& XYZ_2 = mesh_2->location.OBJ;
  vector<vec3>& XYZ_out = mesh_out->location.OBJ;
  XYZ_out.insert( XYZ_out.end(), XYZ_1.begin(), XYZ_1.end());
  XYZ_out.insert( XYZ_out.end(), XYZ_2.begin(), XYZ_2.end());

  //Color
  if(mesh_1->color.hasData && mesh_2->color.hasData){
    vector<vec4>& RGB_1 = mesh_1->color.OBJ;
    vector<vec4>& RGB_2 = mesh_2->color.OBJ;
    vector<vec4>& RGB_out = mesh_out->color.OBJ;

    RGB_out.insert( RGB_out.end(), RGB_1.begin(), RGB_1.end());
    RGB_out.insert( RGB_out.end(), RGB_2.begin(), RGB_2.end());

    mesh_out->color.hasData = true;
  }
  //Normal
  if(mesh_1->normal.hasData && mesh_2->normal.hasData){
    vector<vec3>& Nxyz_1 = mesh_1->normal.OBJ;
    vector<vec3>& Nxyz_2 = mesh_2->normal.OBJ;
    vector<vec3>& Nxyz_out = mesh_out->normal.OBJ;

    Nxyz_out.insert( Nxyz_out.end(), Nxyz_1.begin(), Nxyz_1.end());
    Nxyz_out.insert( Nxyz_out.end(), Nxyz_2.begin(), Nxyz_2.end());

    mesh_out->normal.hasData = true;
  }
  //Intensity
  if(mesh_1->intensity.hasData && mesh_2->intensity.hasData){
    vector<float>& Is_1 = mesh_1->intensity.OBJ;
    vector<float>& Is_2 = mesh_2->intensity.OBJ;
    vector<float>& Is_out = mesh_out->intensity.OBJ;

    Is_out.insert( Is_out.end(), Is_1.begin(), Is_1.end());
    Is_out.insert( Is_out.end(), Is_2.begin(), Is_2.end());

    mesh_out->intensity.hasData = true;
  }

  //Infos
  mesh_out->Name = "Merging_" + mesh_1->Name + "_" + mesh_2->Name;
  mesh_out->Format = ".pts";
  mesh_out->NbPoints = mesh_out->location.OBJ.size();

  //Create slice if any points
  if(mesh_out->NbPoints != 0){
    sceneManager->loadCloud_extracted(mesh_out);
  }
  else{
    cout<<"No points available"<<endl;
  }

  //---------------------------
}
void Extraction::fct_merging_addCloud(Mesh* mesh_1, Mesh* mesh_2){
  //---------------------------

  //Location
  vector<vec3>& XYZ_1 = mesh_1->location.OBJ;
  vector<vec3>& XYZ_2 = mesh_2->location.OBJ;
  XYZ_1.insert( XYZ_1.end(), XYZ_2.begin(), XYZ_2.end());
  mesh_1->location.Initial = XYZ_1;
  mesh_1->NbPoints = XYZ_1.size();

  //Color
  if(mesh_1->color.hasData && mesh_2->color.hasData){
    vector<vec4>& RGB_1 = mesh_1->color.OBJ;
    vector<vec4>& RGB_2 = mesh_2->color.OBJ;
    RGB_1.insert( RGB_1.end(), RGB_2.begin(), RGB_2.end());
    mesh_1->color.Initial = RGB_1;
  }
  //Normal
  if(mesh_1->normal.hasData && mesh_2->normal.hasData){
    vector<vec3>& Nxyz_1 = mesh_1->normal.OBJ;
    vector<vec3>& Nxyz_2 = mesh_2->normal.OBJ;
    Nxyz_1.insert( Nxyz_1.end(), Nxyz_2.begin(), Nxyz_2.end());
    mesh_1->normal.Initial = Nxyz_1;
  }
  //Intensity
  if(mesh_1->intensity.hasData && mesh_2->intensity.hasData){
    vector<float>& Is_1 = mesh_1->intensity.OBJ;
    vector<float>& Is_2 = mesh_2->intensity.OBJ;
    Is_1.insert( Is_1.end(), Is_2.begin(), Is_2.end());
    mesh_1->intensity.Initial = Is_1;
  }

  //---------------------------
}
void Extraction::set_AABB_min(vec3 min_in){
  Mesh* mesh = sceneManager->get_selectedMesh();
  vec3 max_old = mesh->location.Max;
  vec3 min_old = mesh->location.Min;
  //---------------------------

  //Get Z extremums
  sceneManager->update_MinMaxCoords(mesh);
  vec3 min = mesh->location.Min;
  vec3 max = mesh->location.Max;
  vec3 diff = max - min;
  vec3 min_out;

  for(int i=0; i<3; i++){
    if(min_in[i] > 100) min_in[i] = 100;
    if(min_in[i] <= 0) diff[i] = 0;
    else diff[i] = diff[i] * min_in[i]/100;

    min_out[i] = min[i] + diff[i];
    if(min_out[i] > max_old[i]) min_out[i] = max_old[i];
  }

  mesh->location.Max = max_old;
  mesh->location.Min = min_old;
  mesh->location.Min = min_out;

  //---------------------------
  this->fct_highlighting(mesh);
}
void Extraction::set_AABB_max(vec3 max_in){
  Mesh* mesh = sceneManager->get_selectedMesh();
  vec3 max_old = mesh->location.Max;
  vec3 min_old = mesh->location.Min;
  //---------------------------

  //Get Z extremums
  sceneManager->update_MinMaxCoords(mesh);
  vec3 min = mesh->location.Min;
  vec3 max = mesh->location.Max;
  vec3 diff = max - min;
  vec3 max_out;

  for(int i=0; i<3; i++){
    if(max_in[i] > 100) max_in[i] = 100;
    if(max_in[i] <= 0) diff[i] = 0;
    else diff[i] = diff[i] * max_in[i]/100;

    max_out[i] = min[i] + diff[i];
    if(max_out[i] < min_old[i]) max_out[i] = min_old[i];
  }

  mesh->location.Max = max_old;
  mesh->location.Min = min_old;
  mesh->location.Max = max_out;

  //---------------------------
  this->fct_highlighting(mesh);
}

//Selection subparts
void Extraction::fct_selectPart(Mesh* mesh, vec3 mina, vec3 maxa){
  subpart* part = new subpart;
  vec3 max = mesh->location.Max;
  vec3 min = mesh->location.Min;
  //---------------------------

  part->ID = ID_part;
  part->name = to_string(ID_part);
  part->namePC = mesh->Name;
  part->minloc = min;
  part->maxloc = max;

  //---------------------------
  list_part->push_back(part);
  ID_part++;
}
void Extraction::supress_selectedpart(subpart* part){
  //---------------------------

  if(list_part->size() != 0){
    int ID = part->ID;

    list<subpart*>::iterator it = next(list_part->begin(), ID);
    list_part->erase(it);
  }

  //---------------------------
}
