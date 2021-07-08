#ifndef SCENE_H
#define SCENE_H

/**
 * \file Scene.h
 * \brief Point cloud management
 * \author Nathan Sanchiz-Viel
 *
 * Store and update point clouds
 *
 */

class Glyphs;
class Transforms;

#include "../Load/Loader.h"

#include "../Parameters.h"

class Scene
{
public:
  //Constructor / Destructor
  Scene(Glyphs* glyph);
  ~Scene();

public:
  //Loading functions
  Mesh* loadCloud(string pathFile);
  Mesh* loadCloud(string pathFile, Matrix4f realTransformation);
  Mesh* loadCloud(string pathFile, int lmin, int lmax);
  Mesh* loadCloud_extracted(Mesh* mesh);

  void saveCloud(Mesh* mesh, string pathFile);
  void saveCloud_all(string pathDir);
  void removeCloud(Mesh* mesh);
  void removeCloud_all();

  //Update data
  void update_allCloudData(Mesh* mesh);
  void update_CloudPosition(Mesh* mesh);
  void update_CloudColor(Mesh* mesh);
  void update_MinMaxCoords(Mesh* mesh);
  void update_dataFormat(Mesh* mesh);
  void update_IntensityToColor(Mesh* mesh);
  void update_oID(list<Mesh*>* list);
  void update_sortByName(list<Mesh*>* list);
  void update_ResetMesh(Mesh* mesh);
  void update_COM_Initial(Mesh* mesh);
  void update_resetLocation(Mesh* mesh);

  //Subfunctions
  void select_nextMesh();
  void select_specificMesh(int ID);
  void set_MeshVisibility(Mesh* mesh, bool visible);
  void set_selectMeshByName(string name);
  void set_selectedMesh(Mesh* mesh);

  vector<string> get_nameByOrder();
  Mesh* get_otherMesh();
  Mesh* get_MeshByName(string name);
  Mesh* get_MeshByOID(int ID);
  int get_orderSelectedMesh();

  bool is_MeshExist(Mesh* mesh);
  bool is_meshNameExist(Mesh* mesh);
  bool is_atLeastMinNbMesh(int nbMin);

  //Inliner
  inline void set_selectNew(bool value){this->selectNew = value;}
  inline int get_listMeshSize(){return list_Mesh->size();}
  inline Mesh* get_selectedMesh(){return mesh_current;}
  inline list<Mesh*>* get_listMesh(){return list_Mesh;}
  inline Glyphs* get_glyphManager(){return glyphManager;}
  inline bool is_listMeshEmpty(){return list_Mesh->empty();}
  inline bool is_atLeastOneMesh(){return !list_Mesh->empty();}

private:
  //Classes
  Loader loaderManager;
  Glyphs* glyphManager;
  Transforms* transformManager;

  //Pointers on Mesh data
  bool selectNew;
  list<Mesh*>* list_Mesh;
  Mesh* mesh_current;
};

#endif
