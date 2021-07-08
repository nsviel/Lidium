#ifndef POINTCLOUD_STRUCT_H
#define POINTCLOUD_STRUCT_H

struct Location{
  GLuint VBO;

  std::vector<glm::vec3> Initial;
  std::vector<glm::vec3> OBJ;
  std::vector<glm::vec3> Buffer;

  glm::vec3 Min;
  glm::vec3 Max;
  glm::vec3 COM;
  glm::vec3 COM_initial;
  glm::vec3 root;
};
struct Intensity{
  GLuint VBO;
  bool hasData;
  bool heatmap;
  bool corrected;
  bool linearized;

  std::vector<float> Initial;
  std::vector<float> OBJ;
  std::vector<float> Buffer;
};
struct Normal{
  GLuint VBO;
  bool hasData;

  std::vector<glm::vec3> Initial;
  std::vector<glm::vec3> OBJ;
  std::vector<glm::vec3> Buffer;
};
struct Color{
  GLuint VBO;
  bool hasData;

  glm::vec3 uniColor;
  std::vector<glm::vec4> Initial;
  std::vector<glm::vec4> OBJ;
  std::vector<glm::vec4> Buffer;
};
struct Transformation{
  glm::mat4 Scale;
  glm::mat4 Rotation;
  glm::mat4 Translation;
  glm::mat4 TransformationMatrix;
  glm::mat4 RealTransformation;
};
struct Additionnal{
  std::vector<float> dist;
  std::vector<float> cosIt;
  std::vector<float> It;
  std::vector<int> list_selectPoints;
  std::list<int> list_idxPoints;
};
struct Keypoints{
  std::vector<glm::vec3> keypoints;
  std::vector<float> keypoint_weight;
  std::vector<float> keypoint_intensity;
  std::vector<glm::vec3> trgpoints;
  std::vector<glm::vec4> XYZI;
  std::vector<glm::vec3> XYZ_groundTruth;
};
struct PCL{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr XYZRGBN;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZRGB;
  pcl::PointCloud<pcl::PointXYZI>::Ptr XYZI;
  pcl::PointCloud<pcl::Normal>::Ptr Nxyz;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints;
};


//=========================
/**
 * \brief Point cloud structure storing all associated data
 * \struct Mesh struct_pointCloud.h "Point cloud structure"
 */
struct Mesh{
  //Infos
  GLuint ID; //Permanent mesh ID
  GLuint oID; // Order mesh ID
  std::string Name;
  std::string Format;
  std::string dataFormat;
  GLuint VAO;
  GLuint NbPoints;
  bool visibility;
  int pointSize;

  //Data
  Location location;
  Intensity intensity;
  Color color;
  Normal normal;
  Additionnal attribut;
  Transformation transformation;
  Keypoints registration;
  PCL pcl;
};
//=========================

#endif
