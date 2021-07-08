#ifndef FILE_PTS_H
#define FILE_PTS_H

#include "../../Parameters.h"

class filePTS
{
public:
  //Constructor / Destructor
  filePTS();
  ~filePTS();

public:
  //Main functions
  bool Loader(string pathFile);
  bool Exporter(string pathFile, Mesh* mesh);

  //Subfunctions
  bool Loader_parallel(string pathFile);
  bool Loader_subPart(string pathFile, int lmin, int lmax);
  void Loader_init();
  void Loader_nbColumns();
  void Loader_configuration();
  void Loader_data(int FILE_config);
  void Loader_clearData();

  //Loader sub-functions
  bool check_header(string pathFile);
  int check_configuration(string pathFile);
  int check_size(string pathFile, bool FILE_hasHeader);

  inline vector<vec3> get_locationOBJ(){return locationOBJ;}
  inline vector<float> get_intensityOBJ(){return intensityOBJ;}
  inline vector<vec3> get_normalOBJ(){return normalOBJ;}
  inline vector<vec4> get_colorOBJ(){return colorOBJ;}

  inline void set_IdataFormat(int value){this->IdataFormat = value;}
  inline void set_retrievingIntensity(bool value){this->retrieve_I = value;}
  inline void set_retrievingColor(bool value){this->retrieve_RGB = value;}
  inline void set_retrievingNormal(bool value){this->retrieve_N = value;}

private:
  //Variables
  int config;
  int nbptMax;
  int FILE_size, FILE_config;
  bool endHeader, FILE_hasHeader;
  bool endParameters;
  bool hasColor;
  bool hasIntensity;
  bool hasNormal;
  vector<float> line_columns;
  vector<string> dataFormat;
  string line;

  //Parameters
  int IdataFormat, export_IdataFormat;
  bool retrieve_I, retrieve_RGB, retrieve_N;

  //Datatypes
  vector<vec3> locationOBJ;
  vector<vec3> normalOBJ;
  vector<vec4> colorOBJ;
  vector<float> intensityOBJ;
};

#endif
