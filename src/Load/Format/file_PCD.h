#ifndef FILE_PCD_H
#define FILE_PCD_H

#include "../../Parameters.h"

class filePCD
{
public:
  //Constructor / Destructor
  filePCD();
  ~filePCD();

public:
  bool Loader(string pathFile);
  string Loader_header(string pathFile);
  void Loader_XYZ(string pathFile);
  void Loader_XYZI(string pathFile);

  inline vector<vec3> get_locationOBJ(){return locationOBJ;}
  inline vector<float> get_intensityOBJ(){return intensityOBJ;}
  inline vector<vec3> get_normalOBJ(){return normalOBJ;}
  inline vector<vec4> get_colorOBJ(){return colorOBJ;}

private:
  //Datatypes
  vector<vec3> locationOBJ;
  vector<vec3> normalOBJ;
  vector<vec4> colorOBJ;
  vector<float> intensityOBJ;
};

#endif
