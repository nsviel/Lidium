#ifndef FILE_PTX_H
#define FILE_PTX_H

#include "../../Parameters.h"

class filePTX
{
public:
  //Constructor / Destructor
  filePTX();
  ~filePTX();

public:
  //Main functions
  bool Loader(string pathFile);
  bool Exporter(string pathFile);

  //Subfunctions
  void Loader_header(PTXCloud* mesh);
  void Loader_data(PTXCloud* mesh);
  void Loader_assembling();
  void Loader_cloudTransformation();
  void Loader_scannerAtOrigin();

  inline vector<vec3> get_locationOBJ(){return locationOBJ;}
  inline vector<float> get_intensityOBJ(){return intensityOBJ;}
  inline vector<vec3> get_normalOBJ(){return normalOBJ;}
  inline vector<vec4> get_colorOBJ(){return colorOBJ;}

  inline void set_scannerAtOrigin(bool value){this->option_scannerAtOrigin = value;}
  inline void set_applyCloudTransfo(bool value){this->option_applyCloudTransfo = value;}
  inline void set_separateCloud(bool value){this->option_separateCloud = value;}
  inline void set_IdataFormat(int value){this->IdataFormat = value;}
  inline void set_retrievingIntensity(bool value){this->retrieve_I = value;}
  inline void set_retrievingColor(bool value){this->retrieve_RGB = value;}
  inline void set_retrievingNormal(bool value){this->retrieve_N = value;}
  inline void set_notUseZValue(bool value){this->option_notUseZValue = value;}

private:
  //Datatypes
  vector<vec3> locationOBJ;
  vector<vec3> normalOBJ;
  vector<vec4> colorOBJ;
  vector<float> intensityOBJ;

  list<PTXCloud*>* list_ptxCloud;
  float x, y, z, I, r, g ,b;
  int PC_line;

  int IdataFormat;
  bool option_separateCloud;
  bool option_scannerAtOrigin;
  bool option_applyCloudTransfo;
  bool option_notUseZValue;
  bool retrieve_I, retrieve_RGB, retrieve_N;
};

#endif
