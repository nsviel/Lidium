#ifndef DATA_EXTRACTION_H
#define DATA_EXTRACTION_H

#include "../Parameters.h"

class dataExtraction
{
public:
  //Constructor / Destructor
  dataExtraction();
  ~dataExtraction();

public:
  //Main function
  Mesh* extractData(string filePath, vector<vec3>& locationOBJ, vector<float>& intensityOBJ, vector<vec4>& colorOBJ, vector<vec3>& normalOBJ);

  //Subfunctions
  void extract_Location(Mesh* mesh, vector<vec3>& locationOBJ);
  void extract_Color(Mesh* mesh, vector<vec4>& colorOBJ);
  void extract_Normal(Mesh* mesh, vector<vec3>& normalOBJ);
  void extract_Intensity(Mesh* mesh, vector<float>& intensityOBJ);
  void initCloudParameters(Mesh* mesh, string filePath);
  void check_data(Mesh* mesh, vector<vec3>& locationOBJ, vector<float>& intensityOBJ, vector<vec4>& colorOBJ, vector<vec3>& normalOBJ);

private:
  int ID;
  bool isIntensity;
  bool isNormal;
};

#endif
