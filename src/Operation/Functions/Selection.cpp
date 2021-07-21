#include "Selection.h"

#include "../Transforms.h"
#include "CoordTransform.h"
#include "../../Engine/Scene.h"
#include "../../Engine/Glyphs.h"
#include "../../Load/Loader.h"
#include "../Attribut.h"
#include "../../Engine/OpenGL/Camera.h"
#include "../../Engine/Dimension.h"

//Constructor / Destructor
Selection::Selection(Dimension* dim, Scene* scene, Glyphs* glyph, Camera* control){
  this->dimManager = dim;
  this->sceneManager = scene;
  this->glyphManager = glyph;
  this->controlsManager = control;
  //---------------------------

  this->coordTransManager = new CoordTransform(controlsManager, dimManager);
  this->transformManager = new Transforms();
  this->attribManager = new Attribut(sceneManager);

  this->gui_X = 0;
  this->gui_Y = 0;
  this->nbMark = 0;
  this->ID_plane = -1;
  this->selectSensibility = 0.004f;
  this->markMode = "cube";
  this->planeMark = new Mesh();

  //---------------------------
}
Selection::~Selection(){}

//Drawing functions
void Selection::update(){
  //---------------------------

  this->mark_pointLocation();
  //this->mark_planeLocation();

  //---------------------------
}
void Selection::validate(){
  if(sceneManager->is_listMeshEmpty() == false){
    Mesh* mesh = sceneManager->get_selectedMesh();
    list<int>& idx = mesh->attribut.list_idxPoints;
    //---------------------------

    //Plane normal computation
    if(idx.size() == 0){
      this->mark_planeABpoints(mesh);
      float angle = transformManager->make_orientAxis_X(mesh);
      attribManager->compute_normals(mesh);
      vec3 rotation = vec3(0, 0, -angle);
      transformManager->make_rotation(mesh, mesh->location.COM, rotation);
      mesh->normal.Initial = mesh->normal.OBJ;
      sceneManager->update_CloudPosition(mesh);
    }
    //Extract Spectralon parts
    if(idx.size() == 1 && mesh->Name.find("Spectralon") != std::string::npos){
      this->Spectralon_ABpoints(mesh);
      transformManager->make_orientAxis_X(mesh);
      this->Spectralon_extractParts(mesh);
      sceneManager->removeCloud(mesh);
    }
    //Spectralon extraction
    else if(idx.size() == 2){
      this->Spectralon_extractionProcess(mesh);
    }

    //---------------------------
  }
}

//Point selection
void Selection::selectionPoint(vec3 point){
  //---------------------------

  //If selected point already exist, suppress the mark
  bool ptExist = this->mark_pointSupression(point);

  //else create a new mark
  if(!ptExist) this->mark_pointCreation(point);

  //---------------------------
}
void Selection::mark_pointCreation(vec3 point){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  float err = selectSensibility;
  //---------------------------

  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    vector<vec3>& XYZ = mesh->location.OBJ;

    for(int j=0; j<XYZ.size(); j++){
      if(point.x <= XYZ[j].x + err && point.x >= XYZ[j].x - err &&
         point.y <= XYZ[j].y + err && point.y >= XYZ[j].y - err &&
         point.z <= XYZ[j].z + err && point.z >= XYZ[j].z - err){
        vector<float>& Is = mesh->intensity.OBJ;
        const vector<float>& Is_ini = mesh->intensity.Initial;
        vector<float>& It = mesh->attribut.It;

        //Give information about point
        if(It.size() == 0 && mesh->normal.hasData){
          attribManager->compute_cosIt(mesh);
        }
        if(Is.size() != 0 && It.size() != 0){
          float dist = distance(XYZ[j], vec3(0,0,0));
          cout<<XYZ[j].x<<" "<<XYZ[j].y<<" "<<XYZ[j].z;
          cout<<" ("<<dist<<" m)";
          cout<<" -> I_ini= "<<Is_ini[j];
          cout<<" -> I_obj= "<<Is[j];
          cout<<" -> It= "<<It[j]<<endl;
        }else if(Is.size() != 0){
          float dist = distance(point, vec3(0,0,0));
          cout<<XYZ[j].x<<" "<<XYZ[j].y<<" "<<XYZ[j].z;
          cout<<" ("<<dist<<" m)";
          cout<<" -> I= "<<Is[j]<<endl;
        }else{
          float dist = distance(point, vec3(0,0,0));
          cout<<"-> Pt : "<<XYZ[j].x<<" "<<XYZ[j].y<<" "<<XYZ[j].z;
          cout<<" ("<<dist<<" m)"<<endl;
        }

        //Clear point list if no marks
        list<int>& idx = mesh->attribut.list_idxPoints;
        if(list_glyph.size() == 0){
          idx.clear();
        }
        idx.push_back(j);

        if(markMode == "cube"){
          int ID = glyphManager->loadGlyph("../media/engine/Marks/cube.pts", point, "point", false);
          glyphManager->changeColor(ID, vec3(1.0f, 1.0f, 1.0f));
          list_glyph.push_back(ID);
        }
        if(markMode == "sphere"){
          int ID = glyphManager->loadGlyph("../media/engine/Marks/sphere.pts", point, "point", false);
          list_glyph.push_back(ID);
        }

        break;
      }
    }
  }

  //---------------------------
}
bool Selection::mark_pointSupression(vec3 point){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  float err = selectSensibility;
  //---------------------------

  //For each mesh
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    vector<vec3>& XYZ = mesh->location.OBJ;
    list<int>& idx = mesh->attribut.list_idxPoints;

    //If point
    for(int j=0; j<idx.size(); j++){
      int idx_j = *next(idx.begin(), j);
      if(point.x <= XYZ[idx_j].x + err && point.x >= XYZ[idx_j].x - err &&
         point.y <= XYZ[idx_j].y + err && point.y >= XYZ[idx_j].y - err &&
         point.z <= XYZ[idx_j].z + err && point.z >= XYZ[idx_j].z - err){

        list<int>::iterator it_idx = next(idx.begin(), j);
        idx.erase(it_idx);

        int ID = *next(list_glyph.begin(), 0);
        glyphManager->removeGlyph(ID);
        list<int>::iterator it = next(list_glyph.begin(), 0);
        list_glyph.erase(it);

        return true;
      }
    }
  }

  //---------------------------
  return false;
}
void Selection::mark_pointColor(Mesh* ptMark, int num){
  float R, G, B;
  //---------------------------

  switch(num){
    case 0:{
      R = 0.0f;
      G = 0.0f;
      B = 1.0f;
      break;
    }
    case 1:{
      R = 0.0f;
      G = 1.0f;
      B = 0.0f;
      break;
    }
    case 2:{
      R = 1.0f;
      G = 1.0f;
      B = 1.0f;
      break;
    }
    default:{
      R = 1.0f;
      G = 0.0f;
      B = 0.0f;
      break;
    }
  }

  vector<vec4>& RGB = ptMark->color.OBJ;
  for(int i=0; i<RGB.size(); i++){
    RGB[i] = vec4(R, G, B, 1.0f);
  }

  //---------------------------
}
void Selection::mark_supressAll(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);

    list<int>& idx = mesh->attribut.list_idxPoints;
    idx.clear();
  }

  //---------------------------
}
bool Selection::mark_supressSelectedPoints_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  bool atLeastOne = false;
  //---------------------------

  for(int i=0;i<list_Mesh->size();i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    vector<int>& idx = mesh->attribut.list_selectPoints;

    if(idx.size() != 0){
      this->mark_supressSelectedPoints(mesh);
      atLeastOne = true;
    }
  }

  //---------------------------
  return atLeastOne;
}
void Selection::mark_supressSelectedPoints(Mesh* mesh){
  vector<int>& idx = mesh->attribut.list_selectPoints;
  //---------------------------

  if(idx.size() != 0){
    attribManager->make_supressPoints(mesh, idx);
    idx.clear();
  }

  //---------------------------
}
void Selection::mark_pointLocation(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  //Reposionning of ptMark if mesh move
  int cpt = 0;
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    list<int>& idx = mesh->attribut.list_idxPoints;
    vector<vec3>& XYZ = mesh->location.OBJ;

    //Points marks
    if(idx.size() <= list_glyph.size()){
      for(int j=0; j<idx.size(); j++){
        int idx_ = *next(idx.begin(),j);
        int ID = *next(list_glyph.begin(),cpt);

        Glyph* glyph = glyphManager->get_glyphByID(ID);
        glyphManager->update_MinMaxCoords(glyph);
        transformManager->make_positionning_glyph(glyph->location, glyph->COM, XYZ[idx_]);
        glyphManager->update_location(glyph);

        cpt++;
      }
    }
  }

  //If too much ptMark, supress them
  while(cpt < list_glyph.size()){
    int ID = *next(list_glyph.begin(), 0);
    glyphManager->removeGlyph(ID);
    list<int>::iterator it = next(list_glyph.begin(), 0);
    list_glyph.erase(it);
  }

  //---------------------------
}

//Mouse interactivity
vec3 Selection::mouse_clickedPoint(){
  this->update_glDims();
  //---------------------------

  //Get viewport mouse location
  vec2 curPos = dimManager->get_cursorPos();
  vec3 mouse_pos = vec3(curPos.x, gl_Y - curPos.y + gui_Y, 0.0f);
  vec3 point = vec3(-1, -1, -1);

  //Get world location
  glReadPixels(mouse_pos.x, mouse_pos.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &mouse_pos.z);
  if((mouse_pos.z > 0) && (mouse_pos.z < 1)){
    glm::tvec4<unsigned int> viewport(gui_X, 0, gl_X, gl_Y);
    mat4 viewMat = controlsManager->get_viewMat();
    mat4 projMat = controlsManager->get_projMat();
    point = glm::unProject(mouse_pos, viewMat, projMat, viewport);
  }

  //---------------------------
  if(point.x != -1 && point.y != -1 && point.z != -1){
    cout<<"Any selected point.."<<endl;
  }
  return point;
}
void Selection::mouse_selection(int selectMouseMode){
  //---------------------------

  switch(selectMouseMode){
    case 0:{
      this->mouse_meshPicking();
      break;
    }
    case 1:{
      //this->mouse_frameSelection();
      break;
    }
  }

  //---------------------------
}
void Selection::mouse_frameSelection(vec2 point1, vec2 point2){
  //---------------------------

  if(point1.x > point2.x){
    float pt = point1.x;
    point1.x = point2.x;
    point2.x = pt;
  }
  if(point1.y > point2.y){
    float pt = point1.y;
    point1.y = point2.y;
    point2.y = pt;
  }

  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    vector<vec3>& XYZ = mesh->location.OBJ;
    vector<vec4>& RGB = mesh->color.OBJ;
    vector<vec4>& RGB_buf = mesh->color.Buffer;
    vector<int>& idx = mesh->attribut.list_selectPoints;
    idx.clear();

    for(int j=0; j<XYZ.size(); j++){
      vec2 projPT = coordTransManager->WorldToCursor(XYZ[j]);

      if(projPT.x >= point1.x && projPT.y >= point1.y && projPT.x <= point2.x && projPT.y <= point2.y){
        RGB[j] = vec4(1.0f,1.0f,1.0f,1.0f);
        idx.push_back(j);
      }else{
        RGB[j] = RGB_buf[j];
      }
    }

    sceneManager->update_CloudColor(mesh);
  }

  //--------------------------
}
void Selection::mouse_drawFrame(vec2 point1, vec2 point2){
  vec2 point3 = vec2(point1.x, point2.y);
  vec2 point4 = vec2(point2.x, point1.y);
  //---------------------------

  vec3 pt1 = coordTransManager->CursorToWorld(point1);
  vec3 pt2 = coordTransManager->CursorToWorld(point2);
  vec3 pt3 = coordTransManager->CursorToWorld(point3);
  vec3 pt4 = coordTransManager->CursorToWorld(point4);

  //Frame drawing
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
    glVertex3f(pt1.x, pt1.y, pt1.z);
    glVertex3f(pt3.x, pt3.y, pt3.z);

    glVertex3f(pt3.x, pt3.y, pt3.z);
    glVertex3f(pt2.x, pt2.y, pt2.z);

    glVertex3f(pt1.x, pt1.y, pt1.z);
    glVertex3f(pt4.x, pt4.y, pt4.z);

    glVertex3f(pt4.x, pt4.y, pt4.z);
    glVertex3f(pt2.x, pt2.y, pt2.z);
  glEnd();

  //---------------------------
}
void Selection::mouse_meshPicking(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  //---------------------------

  vec3 point = coordTransManager->CursorToGround();
  transformManager->make_positionning_XY(mesh, point);

  //---------------------------
  sceneManager->update_CloudPosition(mesh);
}
vec3 Selection::mouse_cameraPt(){
  vec2 cursorPos = dimManager->get_cursorPos();
  //---------------------------

  //Get ray direction
  float x = (2.0f * cursorPos.x) / gl_X - 1.0f;
  float y = 1.0f - (2.0f * cursorPos.y) / gl_Y;
  float z = 1.0f;
  vec3 ray_nds = vec3(x, y, z);
  vec4 ray_clip = vec4(ray_nds.x, ray_nds.y, -1.0, 1.0);
  mat4 projMat = controlsManager->get_projMat();
  vec4 ray_eye = inverse(projMat) * ray_clip;
  ray_eye = vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);
  mat4 viewMat = controlsManager->get_viewMat();
  vec4 ray_wor = inverse(viewMat) * ray_eye;
  vec3 ray_world = vec3(ray_wor);
  vec3 ray_dir = normalize(ray_world);

  vec3 cam_pos = controlsManager->get_camPos();
  vec3 cam_forw = controlsManager->get_camForward();
  vec3 frustum = cam_pos + vec3(cam_forw.x*0.5,cam_forw.y*0.5,cam_forw.z*0.5);

  vec3 D = ray_dir;
  vec3 N = cam_forw;
  vec3 X = cam_pos + D * dot(frustum - cam_pos, N) / dot(D, N);

  //---------------------------
  return X;
}
void Selection::update_glDims(){
  vec2 glDim = dimManager->get_glDim();
  //---------------------------

  this->gui_X= ImGui::GetWindowSize().x;
  this->gui_Y= ImGui::GetWindowSize().y;
  this->gl_X= glDim.x;
  this->gl_Y= glDim.y;

  //---------------------------
}

//Plane
void Selection::mark_planeCreation(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  list<int>& idx = mesh->attribut.list_idxPoints;
  vector<vec3> XYZ;
  vector<vec4> RGB;
  //---------------------------

  int i0 = *next(idx.begin(), 0);
  int i1 = *next(idx.begin(), 1);
  A = mesh->location.OBJ[i0];
  C = mesh->location.OBJ[i1];

  //infere more points
  B = vec3(C.x, C.y, A.z);
  D = vec3(A.x, A.y, C.z);

  //Plane coordinates
  XYZ.push_back(A);
  XYZ.push_back(B);
  XYZ.push_back(C);
  XYZ.push_back(C);
  XYZ.push_back(D);
  XYZ.push_back(A);

  //Plane color
  for(int i=0; i<XYZ.size(); i++){
    RGB.push_back(vec4(0.0f, 0.0f, 1.0f, 1.0f));
  }

  //---------------------------
  ID_plane = glyphManager->createGlyph(XYZ, RGB, "triangle", false);
}
void Selection::mark_planeABpoints(Mesh* mesh){
  //Retrieve A & B points
  vector<vec3>& XYZ = mesh->location.OBJ;
  A = XYZ[0];
  B = XYZ[0];
  //---------------------------

  for(int i=0; i<XYZ.size(); i++){
    if(XYZ[i].x > A.x){
      A = XYZ[i];
    }
    if(XYZ[i].x < B.x){
      B = XYZ[i];
    }
  }

  //---------------------------
}
void Selection::mark_planeLocation(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  //For each Mesh insert plane
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    list<int>& idx = mesh->attribut.list_idxPoints;
    if(idx.size() == 2){
      this->mark_planeCreation();
    }else if(ID_plane != -1){
      glyphManager->removeGlyph(ID_plane);
    }
  }

  //Reposionning of plane if mesh move
  int cpt = 0;
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    list<int>& idx = mesh->attribut.list_idxPoints;
    vector<vec3>& XYZ = mesh->location.OBJ;

    //Plane marks
    if(ID_plane != -1){
      Glyph* glyph = glyphManager->get_glyphByID(ID_plane);
      vector<vec3>& XYZ_plane = glyph->location;
      if(idx.size() >= 2 && XYZ_plane.size() != 0){
        int i0 = *next(idx.begin(), 0);
        int i1 = *next(idx.begin(), 1);
        vec3 A = XYZ[i0];
        vec3 C = XYZ[i1];
        vec3 B = vec3(C.x, C.y, A.z);
        vec3 D = vec3(A.x, A.y, C.z);

        XYZ_plane[0] = A;
        XYZ_plane[1] = B;
        XYZ_plane[2] = C;
        XYZ_plane[3] = C;
        XYZ_plane[4] = D;
        XYZ_plane[5] = A;

        glyphManager->update_location(glyph);
      }
    }
  }

  //---------------------------
}

//Spectralon
void Selection::Spectralon_extractionProcess(Mesh* mesh){
  //---------------------------

  // 1 -- Extract Spectralon
  this->Spectralon_extractSpectralon(mesh);

  // 2 -- Compute attributs
  this->Spectralon_computeAttibutes(Spectralon);

  // 3 -- Extract Spectralon parts
  this->Spectralon_extractParts(Spectralon);

  //---------------------------
}
void Selection::Spectralon_extractSpectralon(Mesh* mesh){
  list<int>& idx = mesh->attribut.list_idxPoints;
  //---------------------------

  //Align selected plane to Y axis
  int idx_i = *next(idx.begin(), 0);
  int idx_j = *next(idx.begin(), 1);
  A = mesh->location.OBJ[idx_i];
  B = mesh->location.OBJ[idx_j];
  transformManager->make_alignAxisY_AB(mesh, A, B);
  sceneManager->update_CloudPosition(mesh);
  this->mark_planeCreation(); //Reactualize plane coordinates

  //Spectralon geometry
  vector<vec3>& XYZ_plane = planeMark->location.OBJ;
  A = XYZ_plane[0];
  B = XYZ_plane[1];
  C = XYZ_plane[2];
  D = XYZ_plane[4];
  E = vec3(A.x + (B.x - A.x)/2, A.y + (B.y - A.y)/2, A.z + (B.z - A.z)/2);
  F = vec3(D.x + (C.x - D.x)/2, D.y + (C.y - D.y)/2, D.z + (C.z - D.z)/2);
  float X_min = E.x - 0.230 + 0.020;
  float X_max = E.x + 0.230 - 0.020;
  float Y_min = E.y - 0.025;
  float Y_max = E.y + 0.025;
  float Z_min = F.z + 0.020;
  float Z_max = E.z - 0.020;

  //Extract Spectralon
  Mesh* mesh_Spectralon = new Mesh();
  vector<vec3>& XYZ_obj = mesh->location.OBJ;
  vector<vec3>& XYZ_ini = mesh->location.Initial;
  vector<float>& Is = mesh->intensity.OBJ;
  vector<vec3>& XYZ_spe = mesh_Spectralon->location.OBJ;
  vector<float>& Is_spe = mesh_Spectralon->intensity.OBJ;

  for (int i=0; i<XYZ_obj.size(); i++){
    if(XYZ_obj[i].x >= X_min && XYZ_obj[i].x <= X_max &&
      XYZ_obj[i].y >= Y_min && XYZ_obj[i].y <= Y_max &&
      XYZ_obj[i].z >= Z_min && XYZ_obj[i].z <= Z_max){

      XYZ_spe.push_back(XYZ_ini[i]);
      Is_spe.push_back(Is[i]);
    }
  }
  if(XYZ_spe.size() == 0){
    cout<<"No Spectralon points"<<endl;
    return;
  }else{
    cout<<"Spectralon points: "<<XYZ_spe.size()<<endl;
  }

  //Create mesh
  mesh_Spectralon->intensity.hasData = true;
  mesh_Spectralon->color.hasData = false;
  mesh_Spectralon->normal.hasData = false;

  string name = mesh->Name;
  mesh_Spectralon->Name = name + "_";
  mesh_Spectralon->Format = mesh->Format;
  mesh_Spectralon->NbPoints = XYZ_spe.size();

  Spectralon = sceneManager->loadCloud_extracted(mesh_Spectralon);
  sceneManager->removeCloud(mesh);
  Spectralon->Name = name;

  //---------------------------
  cout<<"Spectralon extraction ok"<<endl;
}
void Selection::Spectralon_computeAttibutes(Mesh* mesh){
  //---------------------------

  this->Spectralon_ABpoints(mesh);
  float angle = transformManager->make_alignAxisY_AB(mesh, A, B);
  sceneManager->update_CloudPosition(mesh);
  attribManager->compute_normals_planXaxis(mesh);

  vec3 rotation = vec3(0, 0, -angle);
  transformManager->make_rotation(mesh, mesh->location.COM, rotation);
  mesh->normal.Initial = mesh->normal.OBJ;

  //---------------------------
  cout<<"Spectralon attributs ok"<<endl;
}
void Selection::Spectralon_extractParts(Mesh* mesh){
  vector<vec3>& XYZ_ini = mesh->location.Initial;
  vector<vec3>& XYZ_obj = mesh->location.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<vec3>& Nxyz = mesh->normal.Initial;
  vector<float>& Is = mesh->intensity.Initial;
  //---------------------------

  Mesh* mesh99p = new Mesh();
  Mesh* mesh50p = new Mesh();
  Mesh* mesh25p = new Mesh();
  Mesh* mesh10p = new Mesh();

  transformManager->make_alignAxisY_AB(mesh, A, B);

  //Retrieve middle coordinate
  float X_mean = 0;
  for(int i=0; i<XYZ_obj.size(); i++){
    X_mean += XYZ_obj[i].x;
  }
  X_mean = X_mean / XYZ_obj.size();
  float X_min, X_max;

  for (int i=0; i<XYZ_obj.size(); i++){
    //99%
    X_min = X_mean - 0.230 + 0.020;
    X_max = X_mean - 0.115 - 0.020;
    if(XYZ_obj[i].x >= X_min && XYZ_obj[i].x <= X_max){

      mesh99p->location.OBJ.push_back(XYZ_ini[i]);
      mesh99p->normal.OBJ.push_back(Nxyz[i]);
      mesh99p->intensity.OBJ.push_back(Is[i]);
    }

    //50%
    X_min = X_mean - 0.115 + 0.020;
    X_max = X_mean - 0.020;
    if(XYZ_obj[i].x >= X_min && XYZ_obj[i].x <= X_max){

      mesh50p->location.OBJ.push_back(XYZ_ini[i]);
      mesh50p->normal.OBJ.push_back(Nxyz[i]);
      mesh50p->intensity.OBJ.push_back(Is[i]);
    }

    //25%
    X_min = X_mean + 0.020;
    X_max = X_mean + 0.115 - 0.020;
    if(XYZ_obj[i].x >= X_min && XYZ_obj[i].x <= X_max){

      mesh25p->location.OBJ.push_back(XYZ_ini[i]);
      mesh25p->normal.OBJ.push_back(Nxyz[i]);
      mesh25p->intensity.OBJ.push_back(Is[i]);
    }

    //10%
    X_min = X_mean + 0.115 + 0.020;
    X_max = X_mean + 0.230 - 0.020;
    if(XYZ_obj[i].x >= X_min && XYZ_obj[i].x <= X_max){

      mesh10p->location.OBJ.push_back(XYZ_ini[i]);
      mesh10p->normal.OBJ.push_back(Nxyz[i]);
      mesh10p->intensity.OBJ.push_back(Is[i]);
    }
  }

  //Create areas if any points
  if(mesh99p->location.OBJ.size() != 0){
    mesh99p->Name = mesh->Name + "_99p";
    mesh99p->Format = mesh->Format;
    mesh99p->intensity.hasData = true;
    mesh99p->color.hasData = false;
    mesh99p->normal.hasData = true;

    sceneManager->loadCloud_extracted(mesh99p);
  }else{
    cout<<"99 no points"<<endl;
  }
  if(mesh50p->location.OBJ.size() != 0){
    mesh50p->Name = mesh->Name + "_50p";
    mesh50p->Format = mesh->Format;
    mesh50p->intensity.hasData = true;
    mesh50p->color.hasData = false;
    mesh50p->normal.hasData = true;

    sceneManager->loadCloud_extracted(mesh50p);
  }else{
    cout<<"50 no points"<<endl;
  }
  if(mesh25p->location.OBJ.size() != 0){
    mesh25p->Name = mesh->Name +"_25p";
    mesh25p->Format = mesh->Format;
    mesh25p->intensity.hasData = true;
    mesh25p->color.hasData = false;
    mesh25p->normal.hasData = true;

    sceneManager->loadCloud_extracted(mesh25p);
  }else{
    cout<<"25 no points"<<endl;
  }
  if(mesh10p->location.OBJ.size() != 0){
    mesh10p->Name = mesh->Name + "_10p";
    mesh10p->Format = mesh->Format;
    mesh10p->intensity.hasData = true;
    mesh10p->color.hasData = false;
    mesh10p->normal.hasData = true;

    sceneManager->loadCloud_extracted(mesh10p);
  }else{
    cout<<"10 no points"<<endl;
  }

  //---------------------------
  sceneManager->removeCloud(mesh);
  cout<<"Parts Spectralon extraction ok"<<endl;
}
void Selection::Spectralon_extractParts_generic(Mesh* mesh, string percent, float X_min, float X_max, float X_mean){
  vector<vec3>& XYZ_ini = mesh->location.Initial;
  vector<vec3>& XYZ_obj = mesh->location.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<vec3>& Nxyz = mesh->normal.Initial;
  vector<float>& Is = mesh->intensity.Initial;
  //---------------------------

  Mesh* mesh_p = new Mesh();
  for (int i=0; i<XYZ_obj.size(); i++){
    if(XYZ_obj[i].x >= X_min && XYZ_obj[i].x <= X_max){
      //Location
      mesh_p->location.Initial.push_back(XYZ_ini[i]);
      mesh_p->location.OBJ.push_back(XYZ_ini[i]);
      //Color
      if(mesh->color.hasData){
        mesh_p->color.Initial.push_back(vec4(RGB[i].x, RGB[i].y, RGB[i].z, RGB[i].w));
        mesh_p->color.hasData = false;
      }
      //Normal
      if(mesh->normal.hasData){
        mesh_p->normal.Initial.push_back(Nxyz[i]);
        mesh_p->normal.OBJ.push_back(Nxyz[i]);
        mesh_p->normal.hasData = true;
      }
      //Intensity
      if(mesh->intensity.hasData){
        mesh_p->intensity.Initial.push_back(Is[i]);
        mesh_p->intensity.hasData = true;
      }
    }
  }

  //Create subcloud
  if(mesh_p->location.Initial.size() != 0){
    mesh_p->Name = mesh->Name + "_" + percent + "p";
    mesh_p->Format = mesh->Format;
    sceneManager->loadCloud_extracted(mesh_p);
  }else{
    cout<<percent<<" no points"<<endl;
  }

  //---------------------------
  cout<<"Parts Spectralon extraction ok"<<endl;
}
void Selection::Spectralon_ABpoints(Mesh* mesh){
  //Retrieve A & B points
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  float I_min = Is[0];
  float I_max = Is[0];
  int id_A = 0;
  int id_B = 0;
  //---------------------------

  for(int i=0; i<XYZ.size(); i++){
    if(Is[i] > I_max){
      I_max = Is[i];
      id_A = i;
    }
    if(Is[i] < I_min){
      I_min = Is[i];
      id_B = i;
    }
  }
  A = XYZ[id_A];
  B = XYZ[id_B];

  //---------------------------
}
