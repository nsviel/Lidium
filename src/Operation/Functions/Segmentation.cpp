#include "Segmentation.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/extract_indices.h>

//Constructor / Destructor
Segmentation::Segmentation(){}
Segmentation::~Segmentation(){}

void Segmentation::algo(Mesh* mesh){
  mesh->pcl.XYZRGB = glm_XYZIinit_to_pcl_XYZRGB(mesh);
  mesh->pcl.Nxyz = glm_to_pcl_Nxyz(mesh);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "segmentation..." << std::flush;
  //---------------------------

  // fit plane and keep points above that plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  seg.setInputCloud(mesh->pcl.XYZRGB);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(mesh->pcl.XYZRGB);
  extract.setIndices(inliers);
  extract.setNegative(true);

  extract.filter(*segmented);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);
  std::cout << "OK" << std::endl;

  std::cout << "clustering..." << std::flush;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(segmented);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
  clustering.setClusterTolerance(0.02);
  clustering.setMinClusterSize(1000);
  clustering.setMaxClusterSize(250000);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(segmented);
  clustering.extract(cluster_indices);

  if(cluster_indices.empty()){
    cout<<"No cluster found"<<endl;
    return;
  }else{
    cout << cluster_indices.size() << " clusters found" << endl;
  }

  pcl::IndicesPtr idx(new std::vector<int>);
  *idx = cluster_indices[0].indices;

  extract.setInputCloud(segmented);
  extract.setIndices(idx);
  extract.setNegative(false);
  extract.filter(*segmented);

  //---------------------------
}
