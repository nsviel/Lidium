#include "Fitting.h"

#include "../../Engine/Scene.h"
#include "../../Engine/Glyphs.h"
#include "../Attribut.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

//Constructor / Destructor
Fitting::Fitting(Scene* scene){
  this->sceneManager = scene;
  this->glyphManager = sceneManager->get_glyphManager();
  this->attribManager = new Attribut(sceneManager);
}
Fitting::~Fitting(){}

//Sphere fitting
void Fitting::Sphere_MeshToCenter_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //--------------------------

  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    this->Sphere_MeshToCenter(mesh);
  }
}
void Fitting::Sphere_MeshToCenter(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  if(dist.size() == 0){
    attribManager->compute_Distances(mesh);
  }
  float dist_min = Min(dist);
  int size = XYZ.size();
  float r = 0.0695;
  vec3 Center;
  //---------------------------

  //Search for nearest point
  float distm, Xm, Ym, Zm;
  for (int i=0; i<size; i++){
    if(dist[i] == dist_min){
        distm = dist[i];
        Xm = XYZ[i].x;
        Ym = XYZ[i].y;
        Zm = XYZ[i].z;
    }
  }

  //Determine the center of the sphere
  Center = vec3(Xm + r * (Xm / distm), Ym + r * (Ym / distm), Zm + r * (Zm / distm));
  Center = Sphere_FindCenter(mesh);

  //Add a ptMark mesh to the selected point
  int ID = glyphManager->loadGlyph("../media/engine/Marks/sphere_FARO.pts", Center, "point", false, 3);
  glyphManager->changeColor(ID, vec3(1.0f, 0.0f, 0.0f));

  //---------------------------
}
vec3 Fitting::Sphere_FindCenter(Mesh* mesh){
  /* The return value is 'true' when the linear system of the algorithm
   is solvable, 'false' otherwise. If 'false' is returned, the sphere
   center and radius are set to zero values.*/

  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3 COM = mesh->location.COM;
  vec3 Center;
  int numPoints = XYZ.size();
  //------------------------

  // Compute the covariance matrix M of the Y[i] = X[i]-A and the
  // right-hand side R of the linear system M*(C-A) = R.
  float M00 = 0, M01 = 0, M02 = 0, M11 = 0, M12 = 0, M22 = 0;
  vec3 R = {0, 0, 0};
  for (int i=0; i<numPoints; i++){
     vec3 Y = XYZ[i] - COM;
     float Y0Y0 = Y[0] * Y[0];
     float Y0Y1 = Y[0] * Y[1];
     float Y0Y2 = Y[0] * Y[2];
     float Y1Y1 = Y[1] * Y[1];
     float Y1Y2 = Y[1] * Y[2];
     float Y2Y2 = Y[2] * Y[2];
     M00 += Y0Y0;
     M01 += Y0Y1;
     M02 += Y0Y2;
     M11 += Y1Y1;
     M12 += Y1Y2;
     M22 += Y2Y2;
     R += (Y0Y0 + Y1Y1 + Y2Y2) * Y;
  }
  R *= 0.5;

  // Solve the linear system M*(C-A) = R for the center C.
  float cof00 = M11 * M22 - M12 * M12;
  float cof01 = M02 * M12 - M01 * M22;
  float cof02 = M01 * M12 - M02 * M11;
  float det = M00 * cof00 + M01 * cof01 + M02 * cof02;
  if(det != 0.0f){
     float cof11 = M00 * M22 - M02 * M02;
     float cof12 = M01 * M02 - M00 * M12;
     float cof22 = M00 * M11 - M01 * M01;
     Center[0] = COM[0] + (cof00 * R[0] + cof01 * R[1] + cof02 * R[2]) / det;
     Center[1] = COM[1] + (cof01 * R[0] + cof11 * R[1] + cof12 * R[2]) / det;
     Center[2] = COM[2] + (cof02 * R[0] + cof12 * R[1] + cof22 * R[2]) / det;

     float rsqr = 0.0f;
     for (int i=0; i<numPoints; i++){
         vec3 delta = XYZ[i] - Center;
         rsqr += dotProduct(delta, delta);
     }
     rsqr *= (1/XYZ.size());
     Radius = std::sqrt(rsqr);
  }
  else{
     Center = {0, 0, 0};
     Radius = 0;
  }

  //------------------------
  return Center;
}

//Plane fitting
void Fitting::Plane_Mesh_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //--------------------------

  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    this->Plane_Mesh(mesh);
  }
}
void Fitting::Plane_Mesh(Mesh* mesh){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = glm_to_pcl_XYZ(mesh);
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  int size = mesh->NbPoints;
  //---------------------------

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  vector<int> id;
  for(const auto& idx: inliers->indices){
      id.push_back(idx);
  }

  for(int i=0; i<id.size(); i++){
    RGB[id[i]] = vec4(1.0f, 0.0f, 0.0f, 0.0f);
  }

  //---------------------------
  sceneManager->update_CloudColor(mesh);
}
