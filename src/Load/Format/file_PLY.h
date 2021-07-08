#ifndef FILE_PLY_H
#define FILE_PLY_H

#include "../../Parameters.h"

class filePLY
{
public:
  //Constructor / Destructor
  filePLY();
  ~filePLY();

public:
  //Main functions
  bool Loader(string pathFile);
  bool Exporter(string pathFile, Mesh* mesh);

  //Subfunctions
  void Loader_init();
  void Loader_header();
  void Loader_property();
  void Loader_data();
  void Loader_DoubledPts();

  inline vector<vec3> get_locationOBJ(){return locationOBJ;}
  inline vector<float> get_intensityOBJ(){return intensityOBJ;}
  inline vector<vec3> get_normalOBJ(){return normalOBJ;}
  inline vector<vec4> get_colorOBJ(){return colorOBJ;}

private:
  //Parameters
  int config;
  bool endHeader;
  bool endData;
  bool endProperties;
  bool endCheckProperties;
  bool RGBAlpha;
  bool blenderFile;
  std::string line;

  int cpt;
  int colorColum;
  int normalColumn;
  int reflectanceColumn;
  int intensityColum;
  int positionColumn;
  int nbVertex;

  //Datatypes
  vector<vec3> locationOBJ;
  vector<vec3> normalOBJ;
  vector<vec4> colorOBJ;
  vector<float> intensityOBJ;
};

#endif
