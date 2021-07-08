#ifndef TRANSTYPAGE_FCT_H
#define TRANSTYPAGE_FCT_H

/**
 * \namespace Transtypage
 * \brief Function to convert data format between libraries
 */

namespace{

  //GLM to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr glm_to_pcl_XYZ(Mesh* mesh){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZ> glm_to_pcl_vecXYZ(std::vector<glm::vec3> XYZ){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int size = XYZ.size();
    //---------------------------

    cloud.points.resize (size);
    for(int i=0; i<size; i++){
      cloud.points[i].x = XYZ[i].x;
      cloud.points[i].y = XYZ[i].y;
      cloud.points[i].z = XYZ[i].z;
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr glm_to_pcl_vecXYZ_ptr(std::vector<glm::vec3> XYZ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::Normal>::Ptr glm_to_pcl_Nxyz(Mesh* mesh){
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    const std::vector<glm::vec3>& Nxyz = mesh->normal.OBJ;
    int size = Nxyz.size();
    //---------------------------

    //Normal glm to pcl
    normals->points.resize(size);
    for(int i=0; i<size; i++){
      normals->points[i].normal_x = Nxyz[i].x;
      normals->points[i].normal_y = Nxyz[i].y;
      normals->points[i].normal_z = Nxyz[i].z;
    }

    //---------------------------
    return normals;
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr glm_to_pcl_XYZNxyz(Mesh* mesh){
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    const std::vector<glm::vec3>& Nxyz = mesh->normal.OBJ;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].normal_x = Nxyz[i].x;
      cloud->points[i].normal_y = Nxyz[i].y;
      cloud->points[i].normal_z = Nxyz[i].z;
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr glm_to_pcl_XYZI(Mesh* mesh){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    std::vector<float>& Is = mesh->intensity.OBJ;
    if(Is.size() == 0) std::cout<<"No intensity data for conversion"<<std::endl;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].intensity = Is[i];
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr glm_to_pcl_XYZRGBNormal(Mesh* mesh){
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    const std::vector<glm::vec4>& RGB = mesh->color.Initial;
    const std::vector<glm::vec3>& Nxyz = mesh->normal.OBJ;
    if(RGB.size() == 0) std::cout<<"No color data for conversion"<<std::endl;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].r = (int)(RGB[i].x*255);
      cloud->points[i].g = (int)(RGB[i].y*255);
      cloud->points[i].b = (int)(RGB[i].z*255);

      cloud->points[i].normal_x = Nxyz[i].x;
      cloud->points[i].normal_y = Nxyz[i].y;
      cloud->points[i].normal_z = Nxyz[i].z;
    }

    //---------------------------
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr glm_XYZRGB_to_pcl_XYZRGB(std::vector<glm::vec3> XYZ, std::vector<glm::vec4> RGB){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(RGB.size() == 0) std::cout<<"No color data for conversion"<<std::endl;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].r = (int)(RGB[i].x*255);
      cloud->points[i].g = (int)(RGB[i].y*255);
      cloud->points[i].b = (int)(RGB[i].z*255);
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr glm_XYZRGBobj_to_pcl_XYZRGB(Mesh* mesh){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    const std::vector<glm::vec4>& RGB = mesh->color.OBJ;
    if(RGB.size() == 0) std::cout<<"No color data for conversion"<<std::endl;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].r = (int)(RGB[i].x*255);
      cloud->points[i].g = (int)(RGB[i].y*255);
      cloud->points[i].b = (int)(RGB[i].z*255);
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr glm_XYZRGBinit_to_pcl_XYZRGB(Mesh* mesh){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    const std::vector<glm::vec4>& RGB = mesh->color.Initial;
    if(RGB.size() == 0) std::cout<<"No color data for conversion"<<std::endl;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].r = (int)(RGB[i].x*255);
      cloud->points[i].g = (int)(RGB[i].y*255);
      cloud->points[i].b = (int)(RGB[i].z*255);
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr glm_XYZIobj_to_pcl_XYZRGB(Mesh* mesh){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    std::vector<float>& Is = mesh->intensity.OBJ;
    if(Is.size() == 0) std::cout<<"No intensity data for conversion"<<std::endl;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].r = (int)(Is[i]*255);
      cloud->points[i].g = (int)(Is[i]*255);
      cloud->points[i].b = (int)(Is[i]*255);
    }

    //---------------------------
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr glm_XYZIinit_to_pcl_XYZRGB(Mesh* mesh){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    const std::vector<glm::vec3>& XYZ = mesh->location.OBJ;
    std::vector<float>& Is = mesh->intensity.Initial;
    if(Is.size() == 0) std::cout<<"No intensity data for conversion"<<std::endl;
    int size = XYZ.size();
    //---------------------------

    cloud->points.resize (size);
    for(int i=0; i<size; i++){
      cloud->points[i].x = XYZ[i].x;
      cloud->points[i].y = XYZ[i].y;
      cloud->points[i].z = XYZ[i].z;

      cloud->points[i].r = (int)(Is[i]*255);
      cloud->points[i].g = (int)(Is[i]*255);
      cloud->points[i].b = (int)(Is[i]*255);
    }

    //---------------------------
    return cloud;
  }

  //PCL to GLM
  Mesh pcl_to_glm_XYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    Mesh mesh;
    std::vector<glm::vec3>& XYZ = mesh.location.OBJ;
    int size = cloud->width;
    //---------------------------

    glm::vec3 Point;
    for(int i=0; i<size; i++){
      Point.x = cloud->points[i].x;
      Point.y = cloud->points[i].y;
      Point.z = cloud->points[i].z;

      XYZ.push_back(Point);
    }

    //---------------------------
    return mesh;
  }
  Mesh pcl_to_glm_XYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    Mesh mesh;
    std::vector<glm::vec3>& XYZ = mesh.location.OBJ;
    std::vector<float>& Is = mesh.intensity.OBJ;
    int size = cloud->width;
    //---------------------------

    glm::vec3 Point;
    for(int i=0; i<size; i++){
      Point.x = cloud->points[i].x;
      Point.y = cloud->points[i].y;
      Point.z = cloud->points[i].z;

      XYZ.push_back(Point);
      Is.push_back(cloud->points[i].intensity);
    }

    //---------------------------
    return mesh;
  }
  Mesh pcl_to_glm_XYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    Mesh mesh;
    std::vector<glm::vec3>& XYZ = mesh.location.OBJ;
    std::vector<glm::vec4>& RGB = mesh.color.OBJ;
    int size = cloud->width;
    //---------------------------

    glm::vec3 Point;
    glm::vec4 Color;
    for(int i=0; i<size; i++){
      Point.x = cloud->points[i].x;
      Point.y = cloud->points[i].y;
      Point.z = cloud->points[i].z;

      Color.x = (float)cloud->points[i].r/255.0f;
      Color.y = (float)cloud->points[i].g/255.0f;
      Color.z = (float)cloud->points[i].b/255.0f;
      Color.w = 1.0f;

      XYZ.push_back(Point);
      RGB.push_back(Color);
    }

    //---------------------------
    return mesh;
  }

  std::vector<glm::vec3> pcl_XYZ_to_glm_vecXYZ(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud){
    std::vector<glm::vec3> XYZ;
    glm::vec3 location;
    int size = cloud->size();
    //---------------------------

    for(int i=0; i<size; i++){
      bool a1 = isnan(cloud->points[i].x) == false;
      bool a2 = isnan(cloud->points[i].y) == false;
      bool a3 = isnan(cloud->points[i].z) == false;

      if(a1 && a2 && a3){
        location.x = cloud->points[i].x;
        location.y = cloud->points[i].y;
        location.z = cloud->points[i].z;

        XYZ.push_back(location);
      }
    }

    //---------------------------
    return XYZ;
  }
  std::vector<glm::vec3> pcl_XYZ_to_glm_vecXYZ(pcl::PointCloud <pcl::PointXYZ> cloud){
    std::vector<glm::vec3> XYZ;
    glm::vec3 location;
    int size = cloud.size();
    //---------------------------

    for(int i=0; i<size; i++){
      bool a1 = isnan(cloud.points[i].x) == false;
      bool a2 = isnan(cloud.points[i].y) == false;
      bool a3 = isnan(cloud.points[i].z) == false;

      if(a1 && a2 && a3){
        location.x = cloud.points[i].x;
        location.y = cloud.points[i].y;
        location.z = cloud.points[i].z;

        XYZ.push_back(location);
      }
    }

    //---------------------------
    return XYZ;
  }
  std::vector<glm::vec3> pcl_XYZRGB_to_glm_vecXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    std::vector<glm::vec3> XYZ;
    glm::vec3 location;
    int size = cloud->points.size();
    //---------------------------

    for(int i=0; i<size; i++){
      bool a1 = isnan(cloud->points[i].x) == false;
      bool a2 = isnan(cloud->points[i].y) == false;
      bool a3 = isnan(cloud->points[i].z) == false;
      if(a1 && a2 && a3){
        location.x = cloud->points[i].x;
        location.y = cloud->points[i].y;
        location.z = cloud->points[i].z;

        XYZ.push_back(location);
      }
    }

    //---------------------------
    return XYZ;
  }
  std::vector<glm::vec4> pcl_XYZRGB_to_glm_vecRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    std::vector<glm::vec4> Color;
    glm::vec4 RGB;
    int size = cloud->points.size();
    //---------------------------

    for(int i=0; i<size; i++){
      bool a1 = isnan(cloud->points[i].x) == false;
      bool a2 = isnan(cloud->points[i].y) == false;
      bool a3 = isnan(cloud->points[i].z) == false;
      if(a1 && a2 && a3){
        RGB.x = (float)cloud->points[i].r/255.0f;
        RGB.y = (float)cloud->points[i].g/255.0f;
        RGB.z = (float)cloud->points[i].b/255.0f;
        RGB.w = 1.0f;

        Color.push_back(RGB);
      }
    }

    //---------------------------
    return Color;
  }

  //EIGEN to GLM
  Mesh eigen_to_glm(Eigen::MatrixXf cloud){
    Mesh mesh;
    std::vector<glm::vec3>& XYZ = mesh.location.OBJ;
    //---------------------------

    for(int i=0; i<cloud.rows(); i++){
      for(int j=0; j<cloud.cols(); j++){
        XYZ[i][j] = cloud(i,j);
      }
    }

    //---------------------------
    return mesh;
  }
  glm::mat4 eigen_to_glm_mat4(Eigen::Matrix4f mat_eig){
    glm::mat4 mat_glm;
    //---------------------------

    for(int i=0; i<mat_eig.rows(); i++){
      for(int j=0; j<mat_eig.cols(); j++){
        mat_glm[i][j] = mat_eig(i,j);
      }
    }

    //---------------------------
    return mat_glm;
  }

  //GLM to Eigen
  Eigen::MatrixXf glm_to_eigen(const std::vector<glm::vec3>& XYZ){
    //---------------------------
    int size = XYZ.size();
    Eigen::MatrixXf cloud = Eigen::MatrixXf::Zero(size,3);
    //---------------------------

    for(int i=0; i<size; i++){
      for(int j=0; j<3; j++){
        cloud(i,j) = XYZ[i][j];
      }
    }

    //---------------------------
    return cloud;
  }
  Eigen::MatrixXf glm_to_eigen_mat4(const glm::mat4& mat_glm){
    Eigen::MatrixXf mat_eig(4,4);
    //---------------------------

    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        mat_eig(i,j) = mat_glm[i][j];
      }
    }

    //---------------------------
    return mat_eig;
  }

  //STD to GLM
  glm::mat4 char_to_glm_mat4(char* mat_char){
    //---------------------------

    //Convert char* to string
    std::string str;
    str = mat_char;

    //Convert string to mat4
    std::istringstream iss(str);
    float m0, m1, m2, m3;
    float m4, m5, m6, m7;
    float m8, m9, m10, m11;
    float m12, m13, m14, m15;
    iss >> m0 >> m1 >> m2 >> m3;
    iss >> m4 >> m5 >> m6 >> m7;
    iss >> m8 >> m9 >> m10 >> m11;
    iss >> m12 >> m13 >> m14 >> m15;

    glm::mat4 mat;
    mat[0][0] = m0; mat[0][1] = m1; mat[0][2] = m2; mat[0][3] = m3;
    mat[1][0] = m4; mat[1][1] = m5; mat[1][2] = m6; mat[1][3] = m7;
    mat[2][0] = m8; mat[2][1] = m9; mat[2][2] = m10; mat[2][3] = m11;
    mat[3][0] = m12; mat[3][1] = m13; mat[3][2] = m14; mat[3][3] = m15;

    //---------------------------
    return mat;
  }

}

#endif
