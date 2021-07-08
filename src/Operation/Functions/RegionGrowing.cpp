#include "RegionGrowing.h"

#include "../../Engine/Scene.h"
#include "../Attribut.h"
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

//Constructor / Destructor
RegionGrowing::RegionGrowing(Scene* scene){
  this->sceneManager = scene;
  //---------------------------

  this->attribManager = new Attribut(sceneManager);

  this->thres_dist = 10;
  this->thres_ptColor = 0.02;
  this->thres_regionColor = 0.02;
  this->thres_minClusterSize = 1000;
  this->thres_maxClusterSize = 1000000;
  this->thres_Smoothness = 3;
  this->thres_Curvature = 1.0;

  this->normalsON = true;
  this->intensityON = true;

  //---------------------------
}
RegionGrowing::~RegionGrowing(){}

void RegionGrowing::algo(Mesh* mesh){
  //---------------------------

  if(normalsON && intensityON){
    this->algo_IN_pcl(mesh);
  }
  else if(normalsON && !intensityON){
    this->algo_N_pcl(mesh);
  }
  else if(!normalsON && intensityON){
    this->algo_I_pcl(mesh);
  }

  //---------------------------
}
void RegionGrowing::algo_I_pcl(Mesh* mesh){
  //Récupération de l'intensité
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud  = glm_XYZIobj_to_pcl_XYZRGB(mesh);
  //---------------------------

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(cloud);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(thres_dist);

  //color
  reg.setPointColorThreshold(thres_ptColor*255);
  reg.setRegionColorThreshold(thres_regionColor*255);

  //size
  reg.setMinClusterSize(thres_minClusterSize);
  reg.setMaxClusterSize(thres_maxClusterSize);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  vector<vec4> Color = pcl_XYZRGB_to_glm_vecRGB(colored_cloud);
  mesh->color.OBJ = Color;
  sceneManager->update_CloudColor(mesh);

  //---------------------------
  cout<<"Clustering: "<< clusters.size() <<" clusters"<<endl;
}
void RegionGrowing::algo_N_pcl(Mesh* mesh){
  //Récupération des normales
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud  = glm_to_pcl_XYZ(mesh);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  int size = Nxyz.size();
  //---------------------------

  normals->points.resize(size);
  for(int i=0; i<size; i++){
    normals->points[i].normal_x = Nxyz[i].x;
    normals->points[i].normal_y = Nxyz[i].y;
    normals->points[i].normal_z = Nxyz[i].z;
  }

  pcl::search::Search <pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setInputCloud(cloud);
  reg.setSearchMethod(tree);
  //reg.setNumberOfNeighbours(30);

  //normales
  reg.setInputNormals(normals);
  float angle_rad = (thres_Smoothness * M_PI) / 180.0;
  reg.setSmoothnessThreshold(angle_rad);
  reg.setCurvatureThreshold(thres_Curvature);

  //size
  reg.setMinClusterSize(thres_minClusterSize);
  reg.setMaxClusterSize (1000000);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  vector<vec4> Color = pcl_XYZRGB_to_glm_vecRGB(colored_cloud);
  mesh->color.OBJ = Color;
  sceneManager->update_CloudColor(mesh);

  //---------------------------
  cout<<"Clustering: "<< clusters.size() <<" clusters"<<endl;
}
void RegionGrowing::algo_IN_pcl(Mesh* mesh){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud  = glm_XYZIobj_to_pcl_XYZRGB(mesh);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  int size = Nxyz.size();
  //---------------------------

  normals->points.resize(size);
  for(int i=0; i<size; i++){
    normals->points[i].normal_x = Nxyz[i].x;
    normals->points[i].normal_y = Nxyz[i].y;
    normals->points[i].normal_z = Nxyz[i].z;
  }

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setInputCloud(cloud);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(thres_dist);
  reg.setNumberOfNeighbours(30);

  //normales
  reg.setInputNormals(normals);
  float angle_rad = (thres_Smoothness * M_PI) / 180.0;
  reg.setSmoothnessThreshold(angle_rad);
  reg.setCurvatureThreshold(thres_Curvature);

  //color
  reg.setPointColorThreshold(thres_ptColor*255);
  reg.setRegionColorThreshold(thres_regionColor*255);

  //size
  reg.setMinClusterSize(thres_minClusterSize);
  reg.setMaxClusterSize (1000000);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  vector<vec4> Color = pcl_XYZRGB_to_glm_vecRGB(colored_cloud);
  mesh->color.OBJ = Color;
  sceneManager->update_CloudColor(mesh);

  //---------------------------
  cout<<"Clustering: "<< clusters.size() <<" clusters"<<endl;
}
