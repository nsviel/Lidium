#include "Algo_PCL.h"

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_svd.h>

//Constructor / Destructor
Algo_PCL::Algo_PCL(){}
Algo_PCL::~Algo_PCL(){}

float Algo_PCL::algo_ICP(Mesh* mesh_P, Mesh* mesh_Q){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_XYZ(mesh_P);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_XYZ(mesh_Q);
  //---------------------------

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_P);
  icp.setInputTarget(cloud_Q);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  //---------------------------
  mesh_P->location.OBJ = pcl_XYZ_to_glm_vecXYZ(Final);
  float sse = icp.getFitnessScore();
  return sse;
}
float Algo_PCL::algo_NDT(Mesh* mesh_P, Mesh* mesh_Q){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_XYZ(mesh_P);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_XYZ(mesh_Q);
  //---------------------------

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (cloud_P);
  approximate_voxel_filter.filter (*filtered_cloud);

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon (0.01);
  ndt.setStepSize (0.1);
  ndt.setResolution (1.0);
  ndt.setMaximumIterations (35);
  ndt.setInputSource (filtered_cloud);
  ndt.setInputTarget (cloud_Q);

  //Align
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud);

  //Result
  cout << "Normal Distributions Transform has converged:" << ndt.hasConverged () << flush;
  cout << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*cloud_P, *output_cloud, ndt.getFinalTransformation ());

  //---------------------------
  mesh_P->location.OBJ = pcl_XYZ_to_glm_vecXYZ(output_cloud);
  float sse = ndt.getFitnessScore();
  return sse;
}
float Algo_PCL::algo_GICP(Mesh* mesh_P, Mesh* mesh_Q){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_XYZ(mesh_P);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_XYZ(mesh_Q);
  //---------------------------

  // Initialization
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource (cloud_P);
  gicp.setInputTarget (cloud_Q);

  //Align
  pcl::PointCloud<pcl::PointXYZ> Final;
  gicp.align(Final);

  //---------------------------
  mesh_P->location.OBJ = pcl_XYZ_to_glm_vecXYZ(Final);
  float sse = gicp.getFitnessScore();
  return sse;
}
float Algo_PCL::algo_LUM(Mesh* mesh_P, Mesh* mesh_Q){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_XYZ(mesh_P);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_XYZ(mesh_Q);
  //---------------------------

  // Initializing Normal Distributions Transform (NDT).
  /*pcl::registration::LUM<pcl::PointXYZ, pcl::PointXYZ> lum;
  lum.setInputSource (cloud_P);
  lum.setInputTarget (cloud_Q);

  //Align
  pcl::PointCloud<pcl::PointXYZ> Final;
  lum.align(Final);

  //---------------------------
  mesh_P->location.OBJ = pcl_XYZ_to_glm_vecXYZ(Final);
  float sse = lum.getFitnessScore();
  return sse;*/
  return 0;
}
float Algo_PCL::algo_4ptsCong(Mesh* mesh_P, Mesh* mesh_Q){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_XYZ(mesh_P);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_XYZ(mesh_Q);
  //---------------------------

  // Initializing Normal Distributions Transform (NDT).
  pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> cong;
  cong.setInputSource (cloud_P);
  cong.setInputTarget (cloud_Q);

  //Align
  pcl::PointCloud<pcl::PointXYZ> Final;
  cong.align(Final);

  //---------------------------
  mesh_P->location.OBJ = pcl_XYZ_to_glm_vecXYZ(Final);
  float sse = cong.getFitnessScore();
  return sse;
}

Matrix4f Algo_PCL::optimization_SVD(Mesh* mesh_1, Mesh* mesh_2){
  vector<vec3>& XYZ_P_obj = mesh_1->registration.keypoints;
  vector<vec3>& XYZ_Q_obj = mesh_2->registration.trgpoints;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_vecXYZ_ptr(XYZ_P_obj);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_vecXYZ_ptr(XYZ_Q_obj);
  //---------------------

  Matrix4f transMat;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
  svd.estimateRigidTransformation(*cloud_P, *cloud_Q, transMat);

  //---------------------
  return transMat;
}
Matrix4f Algo_PCL::optimization_LM(Mesh* mesh_1, Mesh* mesh_2){
  vector<vec3>& XYZ_P_obj = mesh_1->registration.keypoints;
  vector<vec3>& XYZ_Q_obj = mesh_2->registration.trgpoints;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_vecXYZ_ptr(XYZ_P_obj);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_vecXYZ_ptr(XYZ_Q_obj);
  //---------------------

  Matrix4f transMat;
  pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> lm;
  lm.estimateRigidTransformation(*cloud_P, *cloud_Q, transMat);

  //---------------------
  return transMat;
}
Matrix4f Algo_PCL::optimization_DualQuaternion(Mesh* mesh_1, Mesh* mesh_2){
  vector<vec3>& XYZ_P_obj = mesh_1->registration.keypoints;
  vector<vec3>& XYZ_Q_obj = mesh_2->registration.trgpoints;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P = glm_to_pcl_vecXYZ_ptr(XYZ_P_obj);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Q = glm_to_pcl_vecXYZ_ptr(XYZ_Q_obj);
  //---------------------

  Matrix4f transMat;
  pcl::registration::TransformationEstimationDualQuaternion<pcl::PointXYZ, pcl::PointXYZ> dq;
  dq.estimateRigidTransformation(*cloud_P, *cloud_Q, transMat);

  //---------------------
  return transMat;
}
