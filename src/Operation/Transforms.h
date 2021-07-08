#ifndef Transforms_H
#define Transforms_H

#include "../Parameters.h"

class Transforms
{
public:
  //Constructor / Destructor
  Transforms();
  ~Transforms();

public:
  //Transformation
  void make_translation(Mesh* mesh, vec3 trans);
  void make_rotation(Mesh* mesh, vec3 COM, vec3 angles);
  void make_scaling(Mesh* mesh, float scaling);

  void make_Transformation(Mesh* mesh, vec3 COM, mat4 transfMat);
  void make_Transformation_atomic(vector<vec3>& XYZ, vec3 COM, mat4 Transformation);
  void make_Transformation_point(vec3& XYZ, vec3 COM, mat4 Transformation);
  void make_Transformation_normal(vector<vec3>& Nxyz, mat4 Transformation);

  //Specific transformation functions
  void make_centering(Mesh* mesh);
  float make_orientAxis_X(Mesh* mesh);
  float make_alignAxisX_AB(Mesh* mesh, vec3 A, vec3 B);
  float make_alignAxisY_AB(Mesh* mesh, vec3 A, vec3 B);
  void make_elevation(Mesh* mesh, float Z);
  void make_alignAxis_X(Mesh* mesh);

  //Positionning functions
  void make_positionning(Mesh* mesh, vec3 pos);
  void make_positionning_XY(Mesh* mesh, vec3 pos);
  void make_positionning_glyph(vector<vec3>& XYZ, vec3& COM, vec3 pos);

  //Operations
  void fct_adjustPosToScanner(Mesh* mesh, float Z_scan);
  float fct_soilDetermination(Mesh* mesh);
  mat4 compute_transformMatrix(Mesh* mesh, vec3 COM, mat4 transformation);
  mat4 compute_transformMatrix(float tx, float ty, float tz, float rx, float ry, float rz);
  vec3 compute_anglesFromTransformationMatrix(const mat4& mat);
  vec3 compute_translFromTransformationMatrix(const mat4& mat);
  vec3 compute_translationsError(Mesh* mesh);
  vec3 compute_anglesError(Mesh* mesh);
  vector<vec3> compute_transformMesh_XYZ(Mesh* mesh, mat4 Mat);
  void compute_transformXYZ(vector<vec3>& XYZ, vec3& COM, mat4 Mat);
  void undo(Mesh* mesh);

  //Setters / Getters
  inline void set_soilNbPoints(int value){this->soilNbPoints = value;}

private:
  int soilNbPoints;
};

#endif
