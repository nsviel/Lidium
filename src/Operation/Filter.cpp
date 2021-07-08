#include "Filter.h"

#include "../Engine/Scene.h"
#include "Attribut.h"
#include <pcl/filters/random_sample.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>

//Constructor / Destructor
Filter::Filter(Scene* scene){
  this->sceneManager = scene;
  //---------------------------

  this->attribManager = new Attribut(sceneManager);

  this->samplingPercent = 50;
  this->outRadiusSearch = 0.1f;
  this->squareSizeSampling = 0.013f;
  this->sampling_std = 1.0f;
  this->sphereDiameter = 0.139f;

  //---------------------------
}
Filter::~Filter(){}

//Functions
void Filter::randSampling(Mesh* mesh){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = glm_to_pcl_XYZ(mesh);
  int size = mesh->NbPoints;
  //---------------------------

  //Retrieve left number of points
  float leftPts = size * (float(samplingPercent)/100);

  //Sampling
  pcl::RandomSample<pcl::PointXYZ> RS(true);
  RS.setInputCloud(cloud);
  RS.setSample(leftPts);
  RS.filter(*cloud);
  pcl::IndicesConstPtr idx = RS.getRemovedIndices();
  vector<int> indices = *idx.get();

  //---------------------------
  attribManager->make_supressPoints(mesh, indices);
  sceneManager->update_allCloudData(mesh);
  cout<<"Sampling : "<<size<<" -> "<<mesh->location.OBJ.size()<<endl;
}
void Filter::spaceSampling_PCL(Mesh* mesh){
  int size = mesh->NbPoints;
  //---------------------------

  //Filtering
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = glm_to_pcl_XYZI(mesh);
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(cloud);
  sor.setDownsampleAllData(true);
  sor.setLeafSize(squareSizeSampling,squareSizeSampling,squareSizeSampling);
  sor.filter(*cloud);

  //Retrieve data
  Mesh mesh_filter = pcl_to_glm_XYZI(cloud);
  mesh->location.OBJ = mesh_filter.location.OBJ;
  mesh->intensity.OBJ = mesh_filter.intensity.OBJ;

  //Update cloud
  sceneManager->update_IntensityToColor(mesh);
  sceneManager->update_allCloudData(mesh);

  //---------------------------
  cout<<"Filtering outliers : "<<size<<" -> "<<mesh->location.OBJ.size()<<endl;
}
void Filter::outlierRemoval(Mesh* mesh){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = glm_to_pcl_XYZ(mesh);
  int size = mesh->NbPoints;
  //---------------------------

  //Filtering
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> RO(true);
  RO.setInputCloud (cloud);
  RO.setRadiusSearch (outRadiusSearch); //0.1m radius search for neigbors
  RO.setMinNeighborsInRadius (1); //min number of neighbors
  RO.setNegative (true);
  RO.filter (*cloud);
  pcl::IndicesConstPtr idx = RO.getRemovedIndices();
  vector<int> indices = *idx.get();

  //---------------------------
  attribManager->make_supressPoints(mesh, indices);
  sceneManager->update_allCloudData(mesh);
  cout<<"Filtering outliers : "<<size<<" -> "<<mesh->location.OBJ.size()<<endl;
}
void Filter::statisticalRemoval(Mesh* mesh){
  //http://pointclouds.org/documentation/tutorials/statistical_outlier.php
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = glm_to_pcl_XYZ(mesh);
  int size = mesh->NbPoints;
  //---------------------------

  //Filtering
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> SO(true);
  SO.setMeanK (8);
  SO.setStddevMulThresh(sampling_std);
  SO.filter (*cloud);
  pcl::IndicesConstPtr idx = SO.getRemovedIndices();
  vector<int> indices = *idx.get();

  //---------------------------
  attribManager->make_supressPoints(mesh, indices);
  sceneManager->update_allCloudData(mesh);
  cout<<"Filtering outliers : "<<size<<" -> "<<mesh->location.OBJ.size()<<endl;
}
void Filter::spaceSampling(Mesh* mesh, float radius){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = glm_to_pcl_XYZ(mesh);
  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3 min = mesh->location.Min;
  vec3 max = mesh->location.Max;
  //---------------------------

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(radius);
  octree.setInputCloud(cloud);
  octree.defineBoundingBox(min[0], min[1], min[2], max[0], max[1], max[2]);
  octree.addPointsFromInputCloud();

  vector<int> id_neighbor, id_pt, id_supp;
  vector<float> ptDist;
  for(int i=0; i<XYZ.size(); i++){
    id_pt.push_back(0);
  }

  //0 : point nont traité
  //-1 : point à supprimer
  //1 : point à garder
  pcl::PointXYZ searchPoint;
  for(int i=0; i<XYZ.size(); i++){
    if(id_pt[i] == 0){
      id_neighbor.clear();
      id_pt[i] = 1;

      searchPoint.x = XYZ[i][0];
      searchPoint.y = XYZ[i][1];
      searchPoint.z = XYZ[i][2];

      octree.radiusSearch(searchPoint, radius, id_neighbor, ptDist);

      for(int j=0; j<id_neighbor.size(); j++){

        if(id_pt[id_neighbor[j]] == 0){
          id_pt[id_neighbor[j]] = -1;
          id_supp.push_back(id_neighbor[j]);
        }
      }
    }
  }

  //---------------------------
  sort(id_supp.begin(), id_supp.end());
  attribManager->make_supressPoints(mesh, id_supp);
  sceneManager->update_allCloudData(mesh);
}
void Filter::filterByAngle(Mesh* mesh, float angleMax){
  attribManager->compute_meshAttributs(mesh);
  vector<float>& It = mesh->attribut.It;
  cout<<"Filtering by angle ("<< angleMax <<"° max) "<<mesh->Name<<" : "<<mesh->NbPoints<<flush;
  //---------------------------

  vector<int> idx;
  for(int i=0; i<It.size(); i++){
    if(It[i] >= angleMax){
      idx.push_back(i);
    }
  }

  //---------------------------
  attribManager->make_supressPoints(mesh, idx);
  cout<<" -> "<<mesh->NbPoints<<endl;
}
void Filter::sphereCleaning_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  float r = sphereDiameter/2;
  float err = r/20;
  //---------------------------

  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    if(mesh->Name.find("Sphere") != std::string::npos){
      vector<vec3>& XYZ = mesh->location.OBJ;
      vector<float>& dist = mesh->attribut.dist;
      if(dist.size() == 0) {attribManager->compute_Distances(mesh);}

      //Search for nearest point
      float distm, Xm, Ym, Zm;
      float dist_min = Min(dist);
      for (int k=0; k<XYZ.size(); k++){
        if(dist[k] == dist_min){
            distm = dist[k];
            Xm = XYZ[k].x;
            Ym = XYZ[k].y;
            Zm = XYZ[k].z;
        }
      }

      //Determine the center of the sphere
      vec3 Center = vec3(Xm + r * (Xm / distm), Ym + r * (Ym / distm), Zm + r * (Zm / distm));

      //For each point supress points too far from radius
      vector<int> idx;
      for(int j=0; j<XYZ.size(); j++){
        float OP = distance(XYZ[j], Center);
        if(OP >= r + err || OP <= r - err){
          idx.push_back(j);
        }
      }

      attribManager->make_supressPoints(mesh, idx);
      sceneManager->update_allCloudData(mesh);
    }
  }

  //---------------------------
}
