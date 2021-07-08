#include "Transforms.h"

//Constructor / Destructor
Transforms::Transforms(){
  //---------------------------

  this->soilNbPoints = 1000;

  //---------------------------
}
Transforms::~Transforms(){}

//Transformation functions
void Transforms::make_translation(Mesh* mesh, vec3 trans){
  //Translation matrice creation
  glm::mat4 translation(1.0);
  //---------------------------

  translation[0][3] = trans.x;
  translation[1][3] = trans.y;
  translation[2][3] = trans.z;

  //---------------------------
  mesh->transformation.Translation *= translation;
  this->make_Transformation(mesh, mesh->location.root, translation);
}
void Transforms::make_rotation(Mesh* mesh, vec3 COM, vec3 angles){
  //Rotation matrice creation - rx, ry, rz are in radian !
  glm::mat4 Rx(1.0);
  glm::mat4 Ry(1.0);
  glm::mat4 Rz(1.0);
  //---------------------------

  float rx = angles.x;
  float ry = angles.y;
  float rz = angles.z;

  Rx[1][1]=cos(rx);
  Rx[2][1]=sin(rx);
  Rx[1][2]=-sin(rx);
  Rx[2][2]=cos(rx);

  Ry[0][0]=cos(ry);
  Ry[0][2]=sin(ry);
  Ry[2][0]=-sin(ry);
  Ry[2][2]=cos(ry);

  Rz[0][0]=cos(rz);
  Rz[1][0]=sin(rz);
  Rz[0][1]=-sin(rz);
  Rz[1][1]=cos(rz);

  glm::mat4 rotation = Rx * Ry * Rz;

  //---------------------------
  mesh->transformation.Rotation *= rotation;
  this->make_Transformation(mesh, COM, rotation);
}
void Transforms::make_scaling(Mesh* mesh, float Sxyz){
  //---------------------------

  //Reverso old scaling
  mat4 scaling_reverse(1/mesh->transformation.Scale[0][0]);
  this->make_Transformation_atomic(mesh->location.OBJ, mesh->location.COM, scaling_reverse);

  //Scale to new value
  mat4 scaling(Sxyz);
  mesh->transformation.Scale = scaling;
  this->make_Transformation_atomic(mesh->location.OBJ, mesh->location.COM, scaling);

  //---------------------------
}

void Transforms::make_Transformation(Mesh* mesh, vec3 COM, mat4 transfMat){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& NXYZ = mesh->normal.OBJ;
  vector<vec3>& KEYP = mesh->registration.keypoints;
  vector<vec3>& KEYT = mesh->registration.trgpoints;
  vec3& ROOT = mesh->location.root;
  //---------------------------

  this->make_Transformation_point(ROOT, COM, transfMat);
  this->make_Transformation_atomic(XYZ, COM, transfMat);
  this->make_Transformation_atomic(KEYP, COM, transfMat);
  this->make_Transformation_atomic(KEYT, COM, transfMat);
  this->make_Transformation_normal(NXYZ, transfMat);

  //---------------------------
  this->compute_transformMatrix(mesh, COM, transfMat);
}
void Transforms::make_Transformation_atomic(vector<vec3>& XYZ, vec3 COM, mat4 Transformation){
  //---------------------------

  //#pragma omp parallel for
  for(int i=0; i<XYZ.size(); i++){
    vec4 XYZ_hom = vec4(XYZ[i].x - COM.x, XYZ[i].y - COM.y, XYZ[i].z - COM.z, 1.0);
    vec4 XYZ_tr = XYZ_hom * Transformation;

    XYZ[i] = vec3(XYZ_tr.x + COM.x, XYZ_tr.y + COM.y, XYZ_tr.z + COM.z);
  }

  //---------------------------
}
void Transforms::make_Transformation_point(vec3& XYZ, vec3 COM, mat4 Transformation){
  //---------------------------

  for(int j=0;j<3;j++) XYZ[j] -= COM[j];
  vec4 newPos = vec4(XYZ, 1.0) * Transformation;
  for(int j=0;j<3;j++) XYZ[j] = newPos[j] + COM[j];

  //---------------------------
}
void Transforms::make_Transformation_normal(vector<vec3>& Nxyz, mat4 Transformation){
  mat3 rotMat = mat3(Transformation);
  //---------------------------

  #pragma omp parallel for
  for(int i=0;i<Nxyz.size();i++){
    Nxyz[i] = Nxyz[i] * rotMat;
  }

  //---------------------------
}

//Specific transformation functions
void Transforms::make_centering(Mesh* mesh){
  vec3 COM = mesh->location.COM;
  //---------------------------

  //Centering cloud
  this->make_translation(mesh, vec3(-COM.x,-COM.y,-COM.z));
  vec3 min = Min_vec3(mesh->location.OBJ);
  this->make_translation(mesh, vec3(0,0,-min[2]));

  //---------------------------
}
void Transforms::make_elevation(Mesh* mesh, float Z){
  vector<vec3>& XYZ_ini = mesh->location.Initial;
  vector<vec3>& XYZ = mesh->location.OBJ;
  //---------------------------

  for(int i=0; i<XYZ.size(); i++){
    XYZ[i].z = XYZ_ini[i].z + Z;
  }

  //---------------------------
}
float Transforms::make_orientAxis_X(Mesh* mesh){
  //---------------------------

  //Retrieve A & B points
  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3 A = XYZ[0];
  vec3 B = XYZ[0];
  for(int i=0; i<XYZ.size(); i++){
    if(XYZ[i].x > A.x){
      A = XYZ[i];
    }
    if(XYZ[i].x < B.x){
      B = XYZ[i];
    }
  }

  //Determination of angle with x axis
  float dot = B.x - A.x;
  float det = B.y - A.y;
  float angle = atan2(det, dot);

  //Align with x axis
  vec3 rotation = vec3(0, 0, angle);
  this->make_rotation(mesh, mesh->location.COM, rotation);

  //---------------------------
  return angle;
}
void Transforms::make_alignAxis_X(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  //---------------------------

  for(int i=0; i<XYZ.size(); i++){
    XYZ[i].x = 0;
  }

  //---------------------------
}
float Transforms::make_alignAxisX_AB(Mesh* mesh, vec3 A, vec3 B){
  //---------------------------

  //Determination of angle with x axis
  float dot = B.x - A.x;
  float det = B.y - A.y;
  float angle = atan2(det, dot);

  //Align with x axis
  vec3 rotation = vec3(0, 0, angle);
  this->make_rotation(mesh, mesh->location.COM, rotation);

  //---------------------------
  return angle;
}
float Transforms::make_alignAxisY_AB(Mesh* mesh, vec3 A, vec3 B){
  //---------------------------

  //Determination of angle with x axis
  float dot = B.x - A.x;
  float det = -(B.y - A.y);
  float angle = atan2(det, dot);

  //Align with x axis
  vec3 rotation = vec3(0, 0, angle);
  this->make_rotation(mesh, mesh->location.COM, rotation);

  //---------------------------
  return angle;
}

//Position functions
void Transforms::make_positionning(Mesh* mesh, vec3 pos){
  vec3& COM = mesh->location.COM;
  //---------------------------

  vec3 diff;
  diff.x = pos.x - COM.x;
  diff.y = pos.y - COM.y;
  diff.z = pos.z - COM.z;

  //---------------------------
  this->make_translation(mesh, diff);
}
void Transforms::make_positionning_XY(Mesh* mesh, vec3 pos){
  vec3& COM = mesh->location.COM;
  //---------------------------

  vec3 diff;
  diff.x = pos.x - COM.x;
  diff.y = pos.y - COM.y;
  diff.z = 0;

  //---------------------------
  this->make_translation(mesh, diff);
}
void Transforms::make_positionning_glyph(vector<vec3>& XYZ, vec3& COM, vec3 pos){
  //---------------------------

  vec3 diff;
  diff.x = pos.x - COM.x;
  diff.y = pos.y - COM.y;
  diff.z = pos.z - COM.z;

  glm::mat4 Tmat(1.0);
  Tmat[0][3] = diff.x;
  Tmat[1][3] = diff.y;
  Tmat[2][3] = diff.z;

  //Application of the Transformation
  for(int i=0;i<XYZ.size();i++){
    //Location
    vec4 XYZ_hom = vec4(XYZ[i], 1.0);
    for(int j=0;j<3;j++){
      XYZ_hom[j] -= COM[j];
    }
    vec4 XYZ_tr = XYZ_hom * Tmat;
    for(int j=0;j<3;j++){
      XYZ_tr[j] += COM[j];
    }
    XYZ[i] = vec3(XYZ_tr.x, XYZ_tr.y, XYZ_tr.z);
  }

  //---------------------------
}

//Operations
float Transforms::fct_soilDetermination(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  //---------------------------

  //Retrieve Z coordinates from XYZ
  vector<float> Z_vec;
  for(int i=0; i<XYZ.size(); i++){
    Z_vec.push_back(XYZ[i].z);
  }

  //Sorting Z vector
  sort(Z_vec.begin(), Z_vec.end());

  //Get the XXX first Z values
  vector<float> Z_fist;
  for(int i=0; i<soilNbPoints; i++){
    Z_fist.push_back(Z_vec[i]);
  }

  //Compute soil estimation by Z mean;
  float Z_soil = fct_Mean(Z_fist);

  //---------------------------
  return Z_soil;
}
void Transforms::fct_adjustPosToScanner(Mesh* mesh, float Z_scan){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3& min = mesh->location.Min;
  float Z_soil = fct_soilDetermination(mesh);
  //---------------------------

  for(int i=0; i<XYZ.size(); i++){
    XYZ[i].z = XYZ[i].z - Z_soil - Z_scan;
  }

  //---------------------------
}
mat4 Transforms::compute_transformMatrix(float tx, float ty, float tz, float rx, float ry, float rz){
  glm::mat4 transMat(1.0);
  glm::mat4 Rx(1.0);
  glm::mat4 Ry(1.0);
  glm::mat4 Rz(1.0);
  //---------------------------

  Rx[1][1]=cos(rx);
  Rx[2][1]=sin(rx);
  Rx[1][2]=-sin(rx);
  Rx[2][2]=cos(rx);

  Ry[0][0]=cos(ry);
  Ry[0][2]=sin(ry);
  Ry[2][0]=-sin(ry);
  Ry[2][2]=cos(ry);

  Rz[0][0]=cos(rz);
  Rz[1][0]=sin(rz);
  Rz[0][1]=-sin(rz);
  Rz[1][1]=cos(rz);

  transMat = Rx * Ry * Rz;
  transMat[0][3] = tx;
  transMat[1][3] = ty;
  transMat[2][3] = tz;

  //---------------------------
  return transMat;
}
mat4 Transforms::compute_transformMatrix(Mesh* mesh, vec3 COM, mat4 transformation){
  mat4& transMat = mesh->transformation.TransformationMatrix;
  //---------------------------

  transMat[0][3] -= COM.x;
  transMat[1][3] -= COM.y;
  transMat[2][3] -= COM.z;

  transMat *= transformation;

  transMat[0][3] += COM.x;
  transMat[1][3] += COM.y;
  transMat[2][3] += COM.z;

  //---------------------------
  return transMat;
}
vec3 Transforms::compute_anglesError(Mesh* mesh){
  mat4 matReal = mesh->transformation.RealTransformation;
  mat4 matIcp = mesh->transformation.TransformationMatrix;
  vec3 angleReal = this->compute_anglesFromTransformationMatrix(matReal);
  vec3 angleIcp = this->compute_anglesFromTransformationMatrix(matIcp);
  //---------------------------

  float ex = angleIcp.x - angleReal.x;
  float ey = angleIcp.y - angleReal.y;
  float ez = angleIcp.z - angleReal.z;

  vec3 error = vec3(ex, ey, ez);

  //---------------------------
  return error;
}
vec3 Transforms::compute_translationsError(Mesh* mesh){
  mat4 matReal = mesh->transformation.RealTransformation;
  mat4 matIcp = mesh->transformation.TransformationMatrix;
  vec3 transReal = this->compute_translFromTransformationMatrix(matReal);
  vec3 transIcp = this->compute_translFromTransformationMatrix(matIcp);
  //---------------------------

  float ex = transIcp.x - transReal.x;
  float ey = transIcp.y - transReal.y;
  float ez = transIcp.z - transReal.z;

  vec3 error = vec3(ex, ey, ez);

  //---------------------------
  return error;
}
vec3 Transforms::compute_anglesFromTransformationMatrix(const mat4& mat){
  vec3 angles;
  //---------------------------

  float ax = atan2(mat[2][1], mat[2][2]);
  float ay = atan2(-mat[2][0], sqrt( pow(mat[2][1], 2) + pow(mat[2][2], 2) ) );
  float az = atan2(mat[1][0], mat[0][0]);

  angles = vec3(ax, ay, az);

  //---------------------------
  return angles;
}
vec3 Transforms::compute_translFromTransformationMatrix(const mat4& mat){
  vec3 translation;
  //---------------------------

  float tx = mat[0][3];
  float ty = mat[1][3];
  float tz = mat[2][3];

  translation = vec3(tx, ty, tz);

  //---------------------------
  return translation;
}
vector<vec3> Transforms::compute_transformMesh_XYZ(Mesh* mesh, mat4 Mat){
  vec3& COM = mesh->location.COM;
  vector<vec3> XYZ = mesh->location.OBJ;
  //---------------------------

  //Application of the Transformation
  for(int i=0;i<XYZ.size();i++){
    vec4 XYZ_hom = vec4(XYZ[i], 1.0);

    for(int j=0;j<3;j++) XYZ_hom[j] -= COM[j];
    vec4 XYZ_tr = XYZ_hom * Mat;
    for(int j=0;j<3;j++) XYZ_tr[j] += COM[j];

    XYZ[i] = vec3(XYZ_tr.x, XYZ_tr.y, XYZ_tr.z);
  }

  //---------------------------
  return XYZ;
}
void Transforms::compute_transformXYZ(vector<vec3>& XYZ, vec3& COM, mat4 Mat){
  //---------------------------

  for(int i=0;i<XYZ.size();i++){
    vec4 XYZ_hom = vec4(XYZ[i], 1.0);

    for(int j=0;j<3;j++){
      XYZ_hom[j] -= COM[j];
    }
    vec4 XYZ_tr = XYZ_hom * Mat;
    for(int j=0;j<3;j++){
      XYZ_tr[j] += COM[j];
    }

    XYZ[i] = vec3(XYZ_tr.x, XYZ_tr.y, XYZ_tr.z);
  }

  //---------------------------
}
void Transforms::undo(Mesh* mesh){
  //---------------------------

  //Retrieve transformation matrix, inverse it and applicate
  this->make_Transformation(mesh, mesh->location.COM, inverse(mesh->transformation.TransformationMatrix));
  mesh->transformation.Translation = mat4(1.0);
  mesh->transformation.Rotation = mat4(1.0);
  mesh->transformation.TransformationMatrix = mat4(1.0);

  //Force to origin
  for(int i=0; i<3; i++){
    if(mesh->location.root[i] < 0.0001){
      mesh->location.root[i]=0;
    }
  }

  //---------------------------
}
