#ifndef Extraction_H
#define Extraction_H

class Scene;
class Attribut;

#include "../../Parameters.h"

class Extraction
{
public:
  //Constructor / Destructor
  Extraction(Scene* scene);
  ~Extraction();

public:
  //Extraction
  void fct_extractCloud(Mesh* mesh);
  void fct_extractSelected(Mesh* mesh);
  void fct_cutCloud(Mesh* mesh);
  void fct_cutCloud_all();
  void fct_highlighting(Mesh* mesh);
  void fct_merging_list(vector<Mesh*> list_part);
  void fct_merging_newCloud(Mesh* mesh_1, Mesh* mesh_2);
  void fct_merging_addCloud(Mesh* mesh_1, Mesh* mesh_2);

  //Selection subparts
  void fct_selectPart(Mesh* mesh, vec3 min, vec3 max);
  void supress_selectedpart(subpart* part);

  //Subfunctions
  void set_AABB_min(vec3 min_in);
  void set_AABB_max(vec3 max_in);

  //Setters / Getters
  inline void set_sliceON(bool value){this->sliceON = value;}
  inline void set_highlightON(bool value){this->highlightON = value;}
  inline list<subpart*>* get_listParts(){return list_part;}

private:
  Scene* sceneManager;
  Attribut* attribManager;

  list<subpart*>* list_part;
  bool highlightON;
  bool sliceON;
  int ID_cloud;
  int ID_part;
};

#endif
