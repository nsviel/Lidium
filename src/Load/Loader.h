#ifndef LOADER_H
#define LOADER_H

#include "Format/file_PTS.h"
#include "Format/file_PLY.h"
#include "Format/file_PTX.h"
#include "Format/file_PCD.h"
#include "dataExtraction.h"

#include "../Parameters.h"

class Loader
{
public:
  //Constructor / Destructor
  Loader();
  ~Loader();

public:
  //Loading functions
  bool load_cloud(string pathFile);
  bool load_cloud_silent(string pathFile);
  bool load_cloud_part(string path, int lmin, int lmax);
  bool load_cloud_creation(Mesh* mesh_in);
  bool load_glyph(string path);
  bool load_binary();
  void load_option(int format, int option, bool value);

  //Saving functions
  bool save_cloud(Mesh* mesh_in, string pathFile);

  //Specific formats
  bool OBJLoader(string pathFile);
  bool PCDLoader(string pathFile);

  inline Mesh* get_createdMesh(){return mesh;}
  inline Glyph* get_createdGlyph(){return glyph;}

private:
  //Subclasses
  filePTS ptsManager;
  filePLY plyManager;
  filePTX ptxManager;
  filePCD pcdManager;
  dataExtraction extractManager;

  //Cloud information
  Mesh* mesh;
  Glyph* glyph;

  //Datatypes
  vector<vec3> locationOBJ;
  vector<vec3> normalOBJ;
  vector<vec4> colorOBJ;
  vector<float> intensityOBJ;
};

#endif
