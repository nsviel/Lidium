#include "Glyphs.h"

#include "../Load/Loader.h"
#include "../Operation/Transforms.h"

//Constructor / Destructor
Glyphs::Glyphs(){
  //---------------------------

  this->transformManager = new Transforms();
  this->loaderManager = new Loader();
  this->list_Glyph = new list<Glyph*>;

  this->match_rdmColor = false;
  this->upColor = false;
  this->normalSize = 1;
  this->gridNbCells = 4;
  this->ID = 0;
  this->ID_sphere = 0;
  this->aabbON = true;
  this->matchON = true;
  this->pointSize = 1;

  //---------------------------
  this->init();
}
Glyphs::~Glyphs(){}

//Main functions
void Glyphs::init(){
  //---------------------------

  this->normalColor = vec3(0.11f,0.35f,0.69f);
  this->aabbColor = vec3(1.0f, 1.0f, 1.0f);
  this->matchingColor = vec3(0.8f, 0.8f, 0.8f);
  this->gridColor = vec3(0.5f, 0.5f, 0.5f);
  this->subgridColor = vec3(0.2f, 0.2f, 0.2f);
  this->planegridColor = vec3(0.15f, 0.15f, 0.15f);

  this->obj_grid();
  this->obj_subgrid();
  this->obj_planegrid();
  this->obj_axis();

  this->declareGlyph("axisMesh", "line", 3, 1);
  this->declareGlyph("aabb", "line", 1, 1);
  this->declareGlyph("axisCircle", "line", 1, 0);
  this->declareGlyph("normal", "line", 1, 0);
  this->declareGlyph("matching", "line", 1, 1);

  //---------------------------
}
void Glyphs::drawing(){
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->visibility){
      glBindVertexArray(glyph->VAO);
      if(glyph->draw_type == "point"){
        glPointSize(glyph->draw_width);
        glDrawArrays(GL_POINTS, 0, glyph->location.size());
      }
      else if(glyph->draw_type == "line"){
        glLineWidth(glyph->draw_width);
        glDrawArrays(GL_LINES, 0, glyph->location.size());
        glLineWidth(1);
      }
      else if(glyph->draw_type == "triangle"){
        glDrawArrays(GL_TRIANGLES, 0, glyph->location.size());
      }
      else if(glyph->draw_type == "quad_strip"){
        glDrawArrays(GL_QUAD_STRIP, 0, glyph->location.size());
      }
      else{
        glDrawArrays(GL_POINTS, 0, glyph->location.size());
      }
      glBindVertexArray(0);
    }
  }

  //---------------------------
}
void Glyphs::clear(){
  //---------------------------

  Glyph* aabb = get_glyphByName("aabb");
  Glyph* axisMesh = get_glyphByName("axisMesh");
  Glyph* normal = get_glyphByName("normal");
  Glyph* matching = get_glyphByName("matching");

  aabb->visibility = false;
  axisMesh->visibility = false;
  normal->visibility = false;
  matching->visibility = false;

  aabb->location.clear();
  axisMesh->location.clear();
  normal->location.clear();
  matching->location.clear();

  //---------------------------
}
void Glyphs::reset(){
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->permanent == false){
      this->removeGlyph(glyph->ID);
      i = 0;
    }
  }

  Glyph* aabb = get_glyphByName("aabb");
  Glyph* axisMesh = get_glyphByName("axisMesh");
  Glyph* normal = get_glyphByName("normal");
  Glyph* matching = get_glyphByName("matching");

  aabb->location.clear();
  axisMesh->location.clear();
  normal->location.clear();
  matching->location.clear();

  //---------------------------
}
void Glyphs::reset_colors(){
  //---------------------------

  this->normalColor = vec3(0.11f,0.35f,0.69f);
  this->aabbColor = vec3(1.0f, 1.0f, 1.0f);
  this->matchingColor = vec3(0.8f, 0.8f, 0.8f);
  this->gridColor = vec3(0.5f, 0.5f, 0.5f);
  this->subgridColor = vec3(0.2f, 0.2f, 0.2f);
  this->planegridColor = vec3(0.15f, 0.15f, 0.15f);

  if(is_glyphByName("aabb")){
    this->update_color(get_glyphByName("aabb"), vec3(1.0f, 1.0f, 1.0f));
  }
  if(is_glyphByName("matching")){
    this->update_color(get_glyphByName("matching"), vec3(0.8f, 0.8f, 0.8f));
  }
  if(is_glyphByName("normal")){
    this->update_color(get_glyphByName("normal"), vec3(0.11f,0.35f,0.69f));
  }

  this->update_color(get_glyphByName("grid"), vec3(0.5f, 0.5f, 0.5f));

  //---------------------------
}

//Glyph update
void Glyphs::update(Mesh* mesh){
  //---------------------------

  this->obj_aabb(mesh);
  this->obj_axisMesh(mesh);
  this->obj_normals(mesh);

  //---------------------------
}
void Glyphs::update_location(Glyph* glyph){
  vector<vec3>& XYZ = glyph->location;
  //---------------------------

  glBindVertexArray(glyph->VAO);
  glBindBuffer(GL_ARRAY_BUFFER, glyph->VBO_location);
  glBufferData(GL_ARRAY_BUFFER, XYZ.size() * sizeof(glm::vec3), &XYZ[0], GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3 * sizeof(float),(void*)0);
  glEnableVertexAttribArray(0);
  glBindVertexArray(0);

  //---------------------------
}
void Glyphs::update_color(Glyph* glyph){
  vector<vec4>& RGB = glyph->color;
  //---------------------------

  glBindVertexArray(glyph->VAO);
  glBindBuffer(GL_ARRAY_BUFFER, glyph->VBO_color);
  glBufferData(GL_ARRAY_BUFFER, RGB.size() * sizeof(glm::vec4), &RGB[0], GL_STATIC_DRAW);
  glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,4 * sizeof(float),(void*)(4* sizeof(float)));
  glEnableVertexAttribArray(3);
  glBindVertexArray(0);

  //---------------------------
}
void Glyphs::update_color(Glyph* glyph, vec3 RGB_in){
  vector<vec4>& RGB = glyph->color;
  //---------------------------

  for(int i=0; i<RGB.size(); i++){
    RGB[i] = vec4(RGB_in, 1.0f);
  }

  glBindVertexArray(glyph->VAO);
  glBindBuffer(GL_ARRAY_BUFFER, glyph->VBO_color);
  glBufferData(GL_ARRAY_BUFFER, RGB.size() * sizeof(glm::vec4), &RGB[0], GL_STATIC_DRAW);
  glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,4 * sizeof(float),(void*)(4* sizeof(float)));
  glEnableVertexAttribArray(3);
  glBindVertexArray(0);

  //---------------------------
}
void Glyphs::update_MinMaxCoords(Glyph* glyph){
  vector<vec3>& XYZ = glyph->location;
  vec3 min = XYZ[0];
  vec3 max = XYZ[0];
  vec3 centroid = vec3(0, 0, 0);
  //---------------------------

  for(int i=0; i<XYZ.size(); i++){
    for(int j=0; j<3; j++){
      if ( XYZ[i][j] <= min[j] ) min[j] = XYZ[i][j];
      if ( XYZ[i][j] >= max[j] ) max[j] = XYZ[i][j];
      centroid[j] += XYZ[i][j];
    }
  }

  for(int j=0;j<3;j++){
    centroid[j] /= XYZ.size();
  }

  //---------------------------
  glyph->Min = min;
  glyph->Max = max;
  glyph->COM = centroid;
}

//Glyph objects
void Glyphs::obj_grid(){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  RGB.push_back(vec4(0.0f, 0.0f, 0.0f, 1.0f));
  int SIZE = gridNbCells;
  //---------------------------

  for(int i=-SIZE; i<=SIZE; i++){
      XYZ.push_back(vec3((float)i, -(float)SIZE, 0));
      XYZ.push_back(vec3((float)i, (float)SIZE, 0));

      XYZ.push_back(vec3(-(float)SIZE, (float)i, 0));
      XYZ.push_back(vec3((float)SIZE, (float)i, 0));

      for(int j=0; j<4; j++){
        RGB.push_back(vec4(gridColor.x, gridColor.y, gridColor.z, 1.0f));
      }
  }

  //Create glyph
  int ID = createGlyph(XYZ, RGB, "line", true);
  Glyph* glyph = get_glyphByID(ID);
  glyph->Name = "grid";
  glyph->draw_width = 1;
  glyph->visibility = true;

  //---------------------------
}
void Glyphs::obj_subgrid(){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  RGB.push_back(vec4(0.0f, 0.0f, 0.0f, 1.0f));
  int SIZE = gridNbCells;
  int SIZE_sg = 10;
  //---------------------------

  int cpt = 0;
  for(int i=-SIZE; i<=SIZE-1; i++){
    for(int j=1; j<SIZE_sg; j++){
        XYZ.push_back(vec3((float)i+(float)j/SIZE_sg, (float)-SIZE, 0));
        XYZ.push_back(vec3((float)i+(float)j/SIZE_sg, (float)SIZE, 0));

        XYZ.push_back(vec3((float)-SIZE, (float)i+(float)j/SIZE_sg, 0));
        XYZ.push_back(vec3((float)SIZE, (float)i+(float)j/SIZE_sg, 0));

        cpt++;
    }
  }

  for(int j=0; j<(cpt*4); j++){
    RGB.push_back(vec4(subgridColor.x, subgridColor.y, subgridColor.z, 1.0f));
  }

  //Create glyph
  int ID = createGlyph(XYZ, RGB, "line", true);
  Glyph* glyph = get_glyphByID(ID);
  glyph->Name = "subgrid";
  glyph->draw_width = 1;
  glyph->visibility = false;

  //---------------------------
}
void Glyphs::obj_planegrid(){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  this->create_Plane(XYZ, RGB, gridNbCells);
  //---------------------------

  int ID = createGlyph(XYZ, RGB, "triangle", true);
  Glyph* glyph = get_glyphByID(ID);
  glyph->Name = "planegrid";
  glyph->draw_width = 1;
  glyph->visibility = false;

  //---------------------------
}
void Glyphs::obj_axis(){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  RGB.push_back(vec4(0.0f, 0.0f, 0.0f, 1.0f));
  //---------------------------

  //X axis
  XYZ.push_back(vec3(0, 0, 0));
  XYZ.push_back(vec3(1, 0, 0));
  RGB.push_back(vec4(1.0f, 0.0f, 0.0f, 1.0f));
  RGB.push_back(vec4(1.0f, 0.0f, 0.0f, 1.0f));

  //Y axis
  XYZ.push_back(vec3(0, 0, 0));
  XYZ.push_back(vec3(0, 1, 0));
  RGB.push_back(vec4(0.0f, 1.0f, 0.0f, 1.0f));
  RGB.push_back(vec4(0.0f, 1.0f, 0.0f, 1.0f));

  //Z axis
  XYZ.push_back(vec3(0, 0, 0));
  XYZ.push_back(vec3(0, 0, 1));
  RGB.push_back(vec4(0.0f, 0.0f, 1.0f, 1.0f));
  RGB.push_back(vec4(0.0f, 0.0f, 1.0f, 1.0f));

  //Create glyph
  int ID = createGlyph(XYZ, RGB, "line", true);
  Glyph* glyph = get_glyphByID(ID);
  glyph->Name = "axis";
  glyph->draw_width = 3;
  glyph->visibility = true;

  //---------------------------
}
void Glyphs::obj_aabb(Mesh* mesh){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  //---------------------------

  //if mesh exist
  RGB.push_back(vec4(0.0f, 0.0f, 0.0f, 1.0f));

  vec3 l1, l2;
  vec3 min = mesh->location.Min;
  vec3 max = mesh->location.Max;

  for(int i=0; i<3; i++){
    l1=min;
    l2=min;
    l2[i]=max[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);


    l1=max;
    l2=max;
    l2[i]=min[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);
  }
  for(int i=0; i<2; i++){
    l1=min;
    l1.z=max.z;
    l2=min;
    l2.z=max.z;
    l2[i]=max[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);

    l1=max;
    l1.z=min.z;
    l2=max;
    l2.z=min.z;
    l2[i]=min[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);

    l1=min;
    l1[i]=max[i];
    l2=l1;
    l2.z=max.z;
    XYZ.push_back(l1);
    XYZ.push_back(l2);
  }
  for(int i=0; i<24; i++){
    RGB.push_back(vec4(aabbColor.x,aabbColor.y,aabbColor.z, 1.0f));
  }

  //Create glyph
  Glyph* glyph = get_glyphByName("aabb");
  glyph->location = XYZ;
  glyph->color = RGB;
  glyph->visibility = aabbON;
  this->update_location(glyph);
  this->update_color(glyph);

  //---------------------------
}
void Glyphs::obj_axisMesh(Mesh* mesh){
  vec3 COM = mesh->location.root;
  vector<vec3> XYZ;
  vector<vec4> RGB;
  RGB.push_back(vec4(0.0f, 0.0f, 0.0f, 1.0f));
  if(mesh->location.OBJ.size() == 0) return;
  //---------------------------

  mat4 rot = mesh->transformation.Rotation;
  vec4 rotX = vec4(0.1,0,0,1) * rot;
  vec4 rotY = vec4(0,0.1,0,1) * rot;
  vec4 rotZ = vec4(0,0,0.1,1) * rot;

  //X Axis
  XYZ.push_back(vec3(COM.x,COM.y,COM.z));
  XYZ.push_back(vec3(COM.x + rotX.x,COM.y + rotX.y,COM.z + rotX.z));
  RGB.push_back(vec4(0.9f, 0.2f, 0.2f, 1.0f));
  RGB.push_back(vec4(0.9f, 0.2f, 0.2f, 1.0f));

  //Y Axis
  XYZ.push_back(vec3(COM.x,COM.y,COM.z));
  XYZ.push_back(vec3(COM.x + rotY.x,COM.y + rotY.y,COM.z + rotY.z));
  RGB.push_back(vec4(0.2f, 0.9f, 0.2f, 1.0f));
  RGB.push_back(vec4(0.2f, 0.9f, 0.2f, 1.0f));

  //Z Axis
  XYZ.push_back(vec3(COM.x,COM.y,COM.z));
  XYZ.push_back(vec3(COM.x + rotZ.x,COM.y + rotZ.y,COM.z + rotZ.z));
  RGB.push_back(vec4(0.2f, 0.2f, 0.9f, 1.0f));
  RGB.push_back(vec4(0.2f, 0.2f, 0.9f, 1.0f));

  //Update glyph
  Glyph* glyph = get_glyphByName("axisMesh");
  glyph->location = XYZ;
  glyph->color = RGB;
  glyph->visibility = true;
  this->update_location(glyph);
  this->update_color(glyph);

  //---------------------------
}
void Glyphs::obj_normals(Mesh* mesh){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  //---------------------------

  vector<vec3>& XYZ_mesh = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  float lgt = 0.01 * normalSize;
  if(XYZ_mesh.size()!=0 && mesh->normal.hasData){
    for(int i=0; i<XYZ_mesh.size(); i++){
      XYZ.push_back(vec3(XYZ_mesh[i].x, XYZ_mesh[i].y, XYZ_mesh[i].z));
      XYZ.push_back(vec3(XYZ_mesh[i].x + Nxyz[i].x * lgt, XYZ_mesh[i].y + Nxyz[i].y * lgt, XYZ_mesh[i].z + Nxyz[i].z * lgt));
      RGB.push_back(vec4(normalColor.x, normalColor.y, normalColor.z, 1.0f));
      RGB.push_back(vec4(normalColor.x, normalColor.y, normalColor.z, 1.0f));
    }
  }

  //Update glyph
  Glyph* glyph = get_glyphByName("normal");
  glyph->location = XYZ;
  glyph->color = RGB;
  this->update_location(glyph);
  this->update_color(glyph);

  //---------------------------
}
void Glyphs::obj_matching(Mesh* mesh_1, Mesh* mesh_2){
  Glyph* glyph = get_glyphByName("matching");
  vector<vec3>& key_P = mesh_1->registration.keypoints;
  vector<vec3>& trg_Q = mesh_2->registration.trgpoints;
  vector<vec3> XYZ;
  vector<vec4> RGB;
  //---------------------------

  if(key_P.size() != 0 && trg_Q.size() != 0){
    RGB.push_back(vec4(0.8f, 0.8f, 0.8f, 1.0));

    for(int i=0; i<key_P.size(); i++){
      //Location
      vec3 xyz_1(key_P[i]);
      vec3 xyz_2(trg_Q[i]);
      XYZ.push_back(xyz_1);
      XYZ.push_back(xyz_2);

      //Color
      float Red, Green, Blue;
      if(key_P.size()*2+1 != glyph->color.size() || upColor){
        if(match_rdmColor){
          Red = float(rand()%101)/100;
          Green = float(rand()%101)/100;
          Blue = float(rand()%101)/100;
        }else{
          Red = matchingColor[0];
          Green = matchingColor[1];
          Blue = matchingColor[2];
        }

        RGB.push_back(vec4(Red, Green, Blue, 1.0));
        RGB.push_back(vec4(Red, Green, Blue, 1.0));

        this->upColor = false;
        glyph->color = RGB;
        this->update_color(glyph);
      }
    }

    //Update glyph
    glyph->location = XYZ;
    glyph->visibility = matchON;
    this->update_location(glyph);
  }

  //---------------------------
}
void Glyphs::obj_axisCircle(float circleRadius){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  int nb_segm = 100;
  RGB.push_back(vec4(1.0f, 1.0f, 1.0f, 1.0f));
  //---------------------------

  int cpt = 0;
  for(int i=0; i<nb_segm; i++){
    float theta_1 = 2.0f * 3.1415926f * float(i) / float(nb_segm);
    float theta_2 = 2.0f * 3.1415926f * float(i+1) / float(nb_segm);

    //Vertex 1
    float x = circleRadius * cosf(theta_1);
    float y = circleRadius * sinf(theta_1);
    XYZ.push_back(vec3(x, y, 0));
    RGB.push_back(vec4(1.0f, 1.0f, 1.0f, 1.0f));

    //Vertex 2
    x = circleRadius * cosf(theta_2);
    y = circleRadius * sinf(theta_2);
    XYZ.push_back(vec3(x, y, 0));
    RGB.push_back(vec4(1.0f, 1.0f, 1.0f, 1.0f));
  }

  //Update glyph
  Glyph* glyph = get_glyphByName("axisCircle");
  glyph->location = XYZ;
  glyph->color = RGB;
  glyph->visibility = true;
  this->update_location(glyph);
  this->update_color(glyph);

  //---------------------------
}
void Glyphs::obj_cube(){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  //---------------------------
/*
  //if mesh exist
  RGB.push_back(vec4(0.0f, 0.0f, 0.0f, 1.0f));

  vec3 l1, l2;
  vec3 min = mesh->location.Min;
  vec3 max = mesh->location.Max;

  for(int i=0; i<3; i++){
    l1=min;
    l2=min;
    l2[i]=max[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);


    l1=max;
    l2=max;
    l2[i]=min[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);
  }
  for(int i=0; i<2; i++){
    l1=min;
    l1.z=max.z;
    l2=min;
    l2.z=max.z;
    l2[i]=max[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);

    l1=max;
    l1.z=min.z;
    l2=max;
    l2.z=min.z;
    l2[i]=min[i];
    XYZ.push_back(l1);
    XYZ.push_back(l2);

    l1=min;
    l1[i]=max[i];
    l2=l1;
    l2.z=max.z;
    XYZ.push_back(l1);
    XYZ.push_back(l2);
  }
  for(int i=0; i<24; i++){
    RGB.push_back(vec4(aabbColor.x,aabbColor.y,aabbColor.z, 1.0f));
  }

  //---------------------------
  int ID = createGlyph(XYZ, RGB, "line", true);
  Glyph* glyph = get_glyphByID(ID);
  glyph->location = XYZ;
  glyph->color = RGB;
  glyph->visibility = true;

  this->update_location(glyph);
  this->update_color(glyph);*/
}
string Glyphs::obj_pointsAtLocation(vector<vec3>& pos, float size, vec4 color){
  vector<vec4> RGB;
  RGB.push_back(color);
  for(int i=0; i<pos.size(); i++){
    RGB.push_back(color);
  }
  //---------------------------

  //Create glyph
  int ID = createGlyph(pos, RGB, "point", true);
  Glyph* glyph = get_glyphByID(ID);
  glyph->Name = "points" + to_string(ID_points);
  glyph->draw_width = size;
  glyph->visibility = true;
  glyph->permanent = false;
  this->update_location(glyph);
  ID_points++;

  //---------------------------
  return glyph->Name;
}
string Glyphs::obj_sphere_RGB(double r, int lats, int longs, vec3 pos, vec3 RGB_in){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  RGB.push_back(vec4(RGB_in.x, RGB_in.y, RGB_in.z, 1.0f));
  //---------------------------

  int i, j;
  for(i = 0; i <= lats; i++){
      double lat0 = M_PI * (-0.5 + (double) (i - 1) / lats);
      double z0  = sin(lat0);
      double zr0 =  cos(lat0);

      double lat1 = M_PI * (-0.5 + (double) i / lats);
      double z1 = sin(lat1);
      double zr1 = cos(lat1);

      for(j = 0; j <= longs; j++) {
          double lng = 2 * M_PI * (double) (j - 1) / longs;
          double x = cos(lng);
          double y = sin(lng);

          XYZ.push_back(vec3(r * x * zr0, r * y * zr0, r * z0));
          XYZ.push_back(vec3(r * x * zr1, r * y * zr1, r * z1));

          RGB.push_back(vec4(RGB_in.x, RGB_in.y, RGB_in.z, 1.0f));
          RGB.push_back(vec4(RGB_in.x, RGB_in.y, RGB_in.z, 1.0f));
      }
  }

  //Create glyph
  int ID = createGlyph(XYZ, RGB, "quad_strip", true);
  Glyph* glyph = get_glyphByID(ID);
  glyph->Name = "sphere" + to_string(ID_sphere);
  glyph->draw_width = 1;
  glyph->visibility = true;
  glyph->permanent = false;
  this->update_MinMaxCoords(glyph);
  transformManager->make_positionning_glyph(glyph->location, glyph->COM, pos);
  this->update_location(glyph);
  ID_sphere++;

  //---------------------------
  return glyph->Name;
}

//Glyph functions
void Glyphs::declareGlyph(string name, string type, float width, bool visible){
  Glyph* glyph = new Glyph();
  //---------------------------

  uint VAO, colorVBO, locationVBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &locationVBO);
  glGenBuffers(1, &colorVBO);
  glyph->VAO = VAO;
  glyph->VBO_location = locationVBO;
  glyph->VBO_color = colorVBO;

  glyph->Name = name;
  glyph->draw_type = type;
  glyph->draw_width = width;
  glyph->visibility = visible;
  glyph->ID = ID++;
  glyph->permanent = true;

  //---------------------------
  list_Glyph->push_back(glyph);
}
void Glyphs::changeColor(int ID, vec3 RGB_new){
  Glyph* glyph = get_glyphByID(ID);
  //---------------------------

  //Reactualise color data
  vector<vec4>& RGB = glyph->color;
  RGB.clear();
  for(int i=0; i<glyph->location.size(); i++){
    RGB.push_back(vec4(RGB_new, 1.0f));
  }

  //---------------------------
  this->update_color(glyph);
}
void Glyphs::removeGlyph(int ID){
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->ID == ID){
      delete glyph;
      list<Glyph*>::iterator it = next(list_Glyph->begin(), i);
      list_Glyph->erase(it);
    }
  }

  //---------------------------
}
void Glyphs::removeGlyph(string ID){
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->Name == ID){
      delete glyph;
      list<Glyph*>::iterator it = next(list_Glyph->begin(), i);
      list_Glyph->erase(it);
    }
  }

  //---------------------------
}
int Glyphs::createGlyph(vector<vec3>& XYZ, vector<vec4>& RGB, string mode, bool perma){
  Glyph* glyph = new Glyph();
  unsigned int VAO;
  uint colorVBO, locationVBO;
  //---------------------------

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  //Vertices
  glGenBuffers(1, &locationVBO);
  glBindBuffer(GL_ARRAY_BUFFER, locationVBO);
  glBufferData(GL_ARRAY_BUFFER, XYZ.size() * sizeof(glm::vec3), &XYZ[0], GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3 * sizeof(float),(void*)0);
  glEnableVertexAttribArray(0);

  //Color
  glGenBuffers(1, &colorVBO);
  glBindBuffer(GL_ARRAY_BUFFER, colorVBO);
  glBufferData(GL_ARRAY_BUFFER, RGB.size() * sizeof(glm::vec4), &RGB[0], GL_STATIC_DRAW);
  glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,4 * sizeof(float),(void*)(4* sizeof(float)));
  glEnableVertexAttribArray(3);

  glyph->VAO = VAO;
  glyph->VBO_location = locationVBO;
  glyph->VBO_color = colorVBO;
  glyph->location = XYZ;
  glyph->color = RGB;

  glyph->Name = "...";
  glyph->draw_type = mode;
  glyph->draw_width = 1;
  glyph->ID = ID++;
  glyph->permanent = perma;

  list_Glyph->push_back(glyph);

  //---------------------------
  return glyph->ID;
}
int Glyphs::loadGlyph(string path, vec3 pos, string mode, bool perma){
  if(loaderManager->load_glyph(path)){
    Glyph* glyph = loaderManager->get_createdGlyph();
    //---------------------------

    glyph->ID = ID++;
    glyph->draw_type = mode;
    glyph->permanent = perma;

    this->update_MinMaxCoords(glyph);
    transformManager->make_positionning_glyph(glyph->location, glyph->COM, pos);
    this->update_MinMaxCoords(glyph);
    this->update_location(glyph);
    list_Glyph->push_back(glyph);

    //---------------------------
    return glyph->ID;
  }else{
    console.AddLog("[error] Problem loading glyph");
    return -1;
  }
}
int Glyphs::loadGlyph(string path, vec3 pos, string mode, bool perma, int point_size){
  if(loaderManager->load_glyph(path)){
    Glyph* glyph = loaderManager->get_createdGlyph();
    //---------------------------

    glyph->ID = ID++;
    glyph->draw_type = mode;
    glyph->draw_width = point_size;
    glyph->permanent = perma;

    this->update_MinMaxCoords(glyph);
    transformManager->make_positionning_glyph(glyph->location, glyph->COM, pos);
    this->update_MinMaxCoords(glyph);
    this->update_location(glyph);
    list_Glyph->push_back(glyph);

    //---------------------------
    return glyph->ID;
  }else{
    console.AddLog("[error] Problem loading glyph");
    return -1;
  }
}
void Glyphs::create_Plane(vector<vec3>& XYZ, vector<vec4>& RGB, int SIZE){
  //---------------------------

  XYZ.push_back(vec3(-SIZE, -SIZE, 0));
  XYZ.push_back(vec3(-SIZE, SIZE, 0));
  XYZ.push_back(vec3(SIZE, SIZE, 0));

  XYZ.push_back(vec3(-SIZE, -SIZE, 0));
  XYZ.push_back(vec3(SIZE, -SIZE, 0));
  XYZ.push_back(vec3(SIZE, SIZE, 0));

  RGB.push_back(vec4(0.0f, 0.0f, 0.0f, 1.0f));
  for(int j=0; j<6; j++){
    RGB.push_back(vec4(planegridColor.x, planegridColor.y, planegridColor.z, 1.0f));
  }

  //---------------------------
}
int Glyphs::loadGlyph_Plane(vec4 coeffs, string mode, bool perma){
  vector<vec3> XYZ;
  vector<vec4> RGB;
  this->create_Plane(XYZ, RGB, 1);
  //---------------------------
/*
  if(loaderManager->load_glyph(path)){
    Glyph* glyph = loaderManager->get_createdGlyph();
    glyph->ID = ID++;
    glyph->draw_type = mode;
    glyph->permanent = perma;

    this->update_MinMaxCoords(glyph);
    transformManager->make_positionning_glyph(glyph->location, glyph->COM, pos);
    this->update_MinMaxCoords(glyph);
    this->update_location(glyph);
    list_Glyph->push_back(glyph);

    return glyph->ID;
  }else{
    cout<<"Problem loading glyph file"<<endl;
    return -1;
  }*/

  //---------------------------
  return 0;
}

//Setters / Getters
bool Glyphs::is_glyphByName(string name){
  bool exist = false;
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->Name == name){
      exist = true;
    }
  }

  //---------------------------
  return exist;
}
bool Glyphs::is_glyphVisible(string name){
  //---------------------------

  Glyph* glyph = get_glyphByName(name);

  //---------------------------
  return glyph->visibility;
}
void Glyphs::set_visibility(string name, bool value){
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->Name == name){
      glyph->visibility = value;
    }
  }

  //---------------------------
}
int Glyphs::get_glyphID(string name){
  int ID_out = -1;
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->Name == name){
      ID_out = glyph->ID;
    }
  }

  //---------------------------
  return ID_out;
}
Glyph* Glyphs::get_glyphByName(string name){
  Glyph* glyph_out;
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->Name == name){
      glyph_out = glyph;
    }
  }

  //---------------------------
  return glyph_out;
}
Glyph* Glyphs::get_glyphByID(int ID){
  Glyph* glyph_out;
  //---------------------------

  for(int i=0;i<list_Glyph->size();i++){
    Glyph* glyph = *next(list_Glyph->begin(),i);

    if(glyph->ID == ID){
      glyph_out = glyph;
    }
  }

  //---------------------------
  return glyph_out;
}
