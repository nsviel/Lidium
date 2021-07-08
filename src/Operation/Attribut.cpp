#include "Attribut.h"

#include "../Engine/Scene.h"
#include "Optimization/Fitting.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

//Constructor / destructor
Attribut::Attribut(Scene* scene){
  this->sceneManager = scene;
  //---------------------------

  this->radiusSearch = 0.03;
  this->sphereRadius = 0.0695;

  //---------------------------
}
Attribut::~Attribut(){}

//General
void Attribut::compute_meshAttributs_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  for(int i=0;i<list_Mesh->size();i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    this->compute_meshAttributs(mesh);
  }

  //---------------------------
}
void Attribut::compute_meshAttributs_list(list<Mesh*>* list){
  //---------------------------

  for(int i=0;i<list->size();i++){
    Mesh* mesh = *next(list->begin(),i);
    this->compute_meshAttributs(mesh);
  }

  //---------------------------
}
void Attribut::compute_meshAttributs(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  vector<float>& cosIt = mesh->attribut.cosIt;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  //---------------------------

  //Distances
  this->compute_Distances(mesh);

  //Normal
  if(Nxyz.size() == 0){
    this->compute_normals(mesh);
  }

  //Incidence angle
  if(Nxyz.size() != 0){
    this->compute_cosIt(mesh);
  }

  //Checking integrity
  this->compute_checkForNan(mesh);

  //---------------------------
  if(Is.size() != XYZ.size()) cout<<"Is data problem - Is: "<<Is.size()<<" XYZ: "<<XYZ.size()<<endl;
  if(dist.size() != XYZ.size()) cout<<"dist data problem"<<endl;
  if(Nxyz.size() != XYZ.size()) cout<<"Nxyz data problem"<<endl;
  if(cosIt.size() != XYZ.size()){
    cout<<"cosit data problem - XYZ: "<<XYZ.size()<<" - cosIt: "<<cosIt.size()<<endl;
  }
}
void Attribut::compute_distToScanner(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  vec3& scanpos = mesh->location.root;
  //---------------------------

  dist.clear();
  for(int i=0; i<XYZ.size(); i++){
    float d = sqrt(pow(XYZ[i].x - scanpos.x,2) + pow(XYZ[i].y - scanpos.y,2) + pow(XYZ[i].z - scanpos.z,2));
    dist.push_back(d);
  }

  //---------------------------
}
void Attribut::compute_Distances(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3 root = mesh->location.root;
  //---------------------------

  //Compute distances
  vector<float> dist(XYZ.size());
  for(int i=0; i<XYZ.size(); i++){
    dist[i] = sqrt(pow(XYZ[i].x - root.x,2) + pow(XYZ[i].y - root.y,2) + pow(XYZ[i].z - root.z,2));
  }

  //---------------------------
  mesh->attribut.dist = dist;
}
void Attribut::make_supressPoints(Mesh* mesh, vector<int>& idx){
  if(idx.size() == 0)return;
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  //Sort indice vector
  sort(idx.begin(), idx.end());

  //Recreate vector -> Fastest delection method
  vector<vec3> XYZ_b;
  vector<vec4> RGB_b;
  vector<float> Is_b;
  vector<vec3> Nxyz_b;
  int cpt = 0;

  for(int i=0; i<XYZ.size(); i++){
    //if i different from not taking account point
    if(i != idx[cpt]){
      XYZ_b.push_back(XYZ[i]);
      if(RGB.size() != 0) RGB_b.push_back(RGB[i]);
      if(Is.size() != 0) Is_b.push_back(Is[i]);
      if(Nxyz.size() != 0) Nxyz_b.push_back(Nxyz[i]);
    }
    //if not taking account point, ok, pass to the next
    else{
      cpt++;
    }
  }

  //location
  mesh->location.OBJ = XYZ_b;
  mesh->location.Buffer = XYZ_b;
  mesh->NbPoints = XYZ_b.size();

  //attributs
  if(RGB.size() != 0){
    mesh->color.OBJ = RGB_b;
    mesh->color.Buffer = RGB_b;
  }
  if(Is.size() != 0){
    mesh->intensity.OBJ = Is_b;
    mesh->intensity.Buffer = Is_b;
  }
  if(Nxyz.size() != 0){
    mesh->normal.OBJ = Nxyz_b;
    mesh->normal.Buffer = Nxyz_b;
  }

  if(mesh->attribut.dist.size() != 0) this->compute_Distances(mesh);
  if(mesh->attribut.cosIt.size() != 0) this->compute_cosIt(mesh);

  //---------------------------
  idx.clear();
  if(XYZ.size() == 0){
    sceneManager->removeCloud(mesh);
  }else{
    sceneManager->update_allCloudData(mesh);
    sceneManager->update_CloudPosition(mesh);
    sceneManager->update_CloudColor(mesh);
  }

  //---------------------------
}
void Attribut::make_supressPoints(vector<vec3>& XYZ, vector<int>& idx){
  if(idx.size() == 0)return;
  //---------------------------

  //Sort indice vector
  sort(idx.begin(), idx.end());

  //Recreate vector -> Fastest delection method
  vector<vec3> XYZ_b;
  int cpt = 0;

  for(int i=0; i<XYZ.size(); i++){
    //if i different from not taking account point
    if(i != idx[cpt]){
      XYZ_b.push_back(XYZ[i]);;
    }
    //if not taking account point, ok, pass to the next
    else{
      cpt++;
    }
  }

  //---------------------------
  idx.clear();
  XYZ = XYZ_b;
}
void Attribut::make_supressPoint(Mesh* mesh, int id){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  //Recreate vector - Fastest delection method
  vector<vec3> XYZ_b, Nxyz_b;
  vector<vec4> RGB_b;
  vector<float> Is_b;

  for(int i=0; i<XYZ.size(); i++){
    if(i != id){
      XYZ_b.push_back(XYZ[i]);
      if(RGB.size() != 0) RGB_b.push_back(RGB[i]);
      if(Is.size() != 0) Is_b.push_back(Is[i]);
      if(Nxyz.size() != 0) Nxyz_b.push_back(Nxyz[i]);
    }
  }

  //location
  mesh->location.OBJ = XYZ_b;
  mesh->location.Buffer = XYZ_b;
  mesh->NbPoints = XYZ_b.size();

  //attributs
  if(RGB.size() != 0){
    mesh->color.OBJ = RGB_b;
    mesh->color.Buffer = RGB_b;
  }
  if(Is.size() != 0){
    mesh->intensity.OBJ = Is_b;
    mesh->intensity.Buffer = Is_b;
  }
  if(Nxyz.size() != 0){
    mesh->normal.OBJ = Nxyz_b;
    mesh->normal.Buffer = Nxyz_b;
  }

  if(mesh->attribut.dist.size() != 0) compute_Distances(mesh);
  if(mesh->attribut.cosIt.size() != 0) this->compute_cosIt(mesh);

  //---------------------------
  mesh->attribut.list_selectPoints.clear();
  if(XYZ.size() == 0){
    sceneManager->removeCloud(mesh);
  }
}
void Attribut::cloudsData(){
  list<Mesh*>* list = sceneManager->get_listMesh();
  //---------------------------

  ofstream myfile;
  myfile.open ("Clouds_Name-Is_Ic_It_cosIt_R.txt");
  myfile << "\n";
  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);

    if(mesh->Name.find("rdm") != std::string::npos){
      vector<float>& Is = mesh->intensity.OBJ;
      vector<float>& Is_ini = mesh->intensity.Initial;
      vector<float>& It = mesh->attribut.It;
      vector<float>& cosIt = mesh->attribut.cosIt;
      vector<float>& dist = mesh->attribut.dist;

      if(cosIt.size() == 0 && mesh->normal.hasData) compute_cosIt(mesh);

      myfile << mesh->Name<< " ";
      myfile << Mean(Is_ini)<<" "<<Mean(Is)<<" "<<Mean(It)<<" "<<Mean(cosIt)<<" "<<Mean(dist);
      myfile << "\n";
    }
  }

  //---------------------------
  myfile.close();
  cout<<"Data extracted in Clouds_Name-*.txt"<<endl;
}

//Color
void Attribut::set_pointCloudColor(Mesh* mesh, vec4 RGBA){
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  for(int i=0; i<RGB.size(); i++){
    if(Is.size() != 0){
      RGB[i] = vec4(RGBA.x*Is[i] , RGBA.y*Is[i], RGBA.z*Is[i], RGBA.w);
    }else{
      RGB[i] = vec4(RGBA.x , RGBA.y, RGBA.z, RGBA.w);
    }
  }

  //---------------------------
  sceneManager->update_CloudColor(mesh);
}
void Attribut::set_colorRGB_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  for(int i=0;i<list_Mesh->size();i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    this->set_colorRGB(mesh);
  }

  //---------------------------
}
void Attribut::set_colorRGB(Mesh* mesh){
  vector<vec4>& RGB_obj = mesh->color.OBJ;
  vector<vec4>& RGB_buf = mesh->color.Initial;
  //---------------------------

  RGB_obj = RGB_buf;

  //---------------------------
  sceneManager->update_CloudColor(mesh);
}
void Attribut::set_colorI_all(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  //---------------------------

  for(int i=0;i<list_Mesh->size();i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    this->set_colorI(mesh);
  }

  //---------------------------
}
void Attribut::set_colorI(Mesh* mesh){
  vector<vec4>& RGB_obj = mesh->color.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  RGB_obj.clear();
  for(int i=0; i<Is.size(); i++){
    RGB_obj.push_back(vec4(Is[i], Is[i], Is[i], 1.0f));
  }

  //---------------------------
  sceneManager->update_CloudColor(mesh);
}
void Attribut::set_enhancedColor(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  const vector<vec4>& RGB = mesh->color.Initial;
  //---------------------------

  for(int i=0; i<RGB.size(); i++){
    vec4 rgb = RGB[i];
    float& I = Is[i];
    rgb = vec4(rgb.x*I, rgb.y*I, rgb.z*I, 1.0f);
  }

  //---------------------------
  sceneManager->update_CloudColor(mesh);
}
void Attribut::set_pointsRandomColor(Mesh* mesh){
  vector<vec4>& RGB = mesh->color.OBJ;
  //---------------------------

  float Red, Green, Blue;
  for(int i=0; i<RGB.size(); i++){
    Red = float(rand()%101)/100;
    Green = float(rand()%101)/100;
    Blue = float(rand()%101)/100;

    RGB[i] = vec4(Red, Green, Blue, 1.0f);
  }

  //---------------------------
  mesh->color.Initial = RGB;
}
void Attribut::set_restoreInitialColor(Mesh* mesh){
  vector<vec4>& RGB_o = mesh->color.OBJ;
  const vector<vec4>& RGB_i = mesh->color.Initial;
  //---------------------------

  RGB_o = RGB_i;
  mesh->color.Buffer = RGB_o;

  //---------------------------
  sceneManager->update_CloudColor(mesh);
}

//Normal
void Attribut::compute_normals(Mesh* mesh){
  //---------------------------

  if(mesh->Name.find("Sphere") != std::string::npos || mesh->Name.find("sphere") != std::string::npos){
    this->compute_sphereNormals(mesh);
  }else if(mesh->Name.find("Spectralon") != std::string::npos || mesh->Name.find("spectralon") != std::string::npos){
    this->compute_planeNormals_fitting(mesh);
  }else{
    this->compute_normalPCL(mesh);
  }

  //---------------------------
}
void Attribut::compute_normalPCL(Mesh* mesh){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = glm_to_pcl_XYZ(mesh);
  tic();
  //---------------------------

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (radiusSearch);

  // /!\ compute the normales with direction to the scanner position
  vec3& scanpos = mesh->location.root;
  ne.setViewPoint(scanpos.x, scanpos.y, scanpos.z);

  // Compute the features
  ne.compute (*cloud_normals);

  //Store computed normals into cloud data
  vec3 normal;
  mesh->normal.OBJ.clear();
  #pragma omp parallel for
  for(int i=0;i<mesh->location.OBJ.size();i++){
    normal.x = cloud_normals->points[i].normal_x;
    normal.y = cloud_normals->points[i].normal_y;
    normal.z = cloud_normals->points[i].normal_z;

    mesh->normal.OBJ.push_back(normal);
  }
  mesh->normal.Initial = mesh->normal.OBJ;
  mesh->normal.hasData = true;

  //---------------------------
  float duration = toc();
  console.AddLog("Normal for %s computed in %.2f s", mesh->Name.c_str(), duration);
}
void Attribut::compute_sphereNormals(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  tic();
  //---------------------------

  //Check data
  if(dist.size() == 0) {compute_Distances(mesh);}

  //Search for nearest point
  float distm, Xm, Ym, Zm;
  float dist_min = Min(dist);
  for (int i=0; i<XYZ.size(); i++){
    if(dist[i] == dist_min){
        distm = dist[i];
        Xm = XYZ[i].x;
        Ym = XYZ[i].y;
        Zm = XYZ[i].z;
    }
  }

  //Determine the center of the sphere
  Fitting fitManager(sceneManager);
  vec3 Center = fitManager.Sphere_FindCenter(mesh);

  //Compute normals
  vector<vec3> Nxyz(XYZ.size());
  for (int i=0; i<XYZ.size(); i++){
    for (int j=0; j<3; j++){
      Nxyz[i][j] = (XYZ[i][j] - Center[j]) / sphereRadius;
    }
  }
  mesh->normal.Initial = Nxyz;
  mesh->normal.OBJ = Nxyz;
  mesh->normal.hasData = true;

  //---------------------------
  float duration = toc();
  console.AddLog("Normal for %s computed in %.2f s", mesh->Name.c_str(), duration);
}
void Attribut::compute_planeNormals_Xaxis(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vec3 norm, Point;
  Nxyz.clear();

  vec3 P1 = vec3(10, 0, 10);
  vec3 P2 = vec3(-10, 0, -10);
  //-------------------------

  //Compute normal to a plane for each points
  for(int i=0; i<XYZ.size(); i++){
    //(glm::cross(c - a, b - a))
    Point = vec3(XYZ[i].x, 0, XYZ[i].z);
    norm = glm::normalize(glm::cross(Point - P1, P2 - P1));
    Nxyz.push_back(norm);
  }

  //-------------------------
  mesh->normal.Initial = Nxyz;
  mesh->normal.hasData = true;
}
void Attribut::compute_planeNormals_Yaxis(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vec3 norm, Point;
  Nxyz.clear();
  //---------------------------

  vec3 P1 = vec3(0, 10, 10);
  vec3 P2 = vec3(0, -10, -10);

  //Compute normal to a plane for each points
  for(int i=0; i<XYZ.size(); i++){
    //(glm::cross(c - a, b - a))
    Point = vec3(0, XYZ[i].y, XYZ[i].z);
    norm = glm::normalize(glm::cross(Point - P1, P2 - P1));
    Nxyz.push_back(norm);
  }

  //-------------------------
  mesh->normal.Initial = Nxyz;
  mesh->normal.hasData = true;
}
void Attribut::compute_planeNormals_Zaxis(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vec3 norm, Point;
  Nxyz.clear();
  //---------------------------

  vec3 P1 = vec3(100, 100, 0);
  vec3 P2 = vec3(100, -100, 0);

  //Compute normal to a plane for each points
  for(int i=0; i<XYZ.size(); i++){
    //(glm::cross(c - a, b - a))
    Point = vec3(XYZ[i].x, XYZ[i].y, 0);
    norm = glm::normalize(glm::cross(Point - P1, P2 - P1));
    Nxyz.push_back(norm);
  }

  //-------------------------
  mesh->normal.Initial = Nxyz;
  mesh->normal.hasData = true;
}
void Attribut::compute_planeNormals_fitting(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  tic();
  //---------------------------

	// copy coordinates to  matrix in Eigen format
	Eigen::Matrix< Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, XYZ.size());
	for(int i=0; i<XYZ.size(); i++){
    for(int j=0; j<3; j++){
      coord(j,i) = XYZ[i][j];
    }
  }

	// calculate centroid
	Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

	// subtract centroid
	coord.row(0).array() -= centroid(0);
  coord.row(1).array() -= centroid(1);
  coord.row(2).array() -= centroid(2);

	// we only need the left-singular matrix here
	auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	Vector3f plane_normal = svd.matrixU().rightCols<1>();

  //Store normal data
  Nxyz.clear();
  for(int i=0; i<XYZ.size(); i++){
    Nxyz.push_back(vec3(plane_normal(0),plane_normal(1),plane_normal(2)));
  }
  mesh->normal.hasData = true;

  //Reoriente normal in the origin direction
  this->compute_normalReorientation(mesh);

  //---------------------------
  float duration = toc();
  console.AddLog("Normal for %s computed in %.2f s", mesh->Name.c_str(), duration);
}
void Attribut::compute_normalInversion(){
  if(sceneManager->is_atLeastOneMesh()){
    Mesh* mesh = sceneManager->get_selectedMesh();
    vector<vec3>& normals = mesh->normal.OBJ;
    //---------------------------

    for(int i=0; i<normals.size(); i++){
      for(int j=0; j<3; j++){
        normals[i][j] = -normals[i][j];
      }
    }
    mesh->normal.Initial = mesh->normal.OBJ;

    //---------------------------
  }
}
void Attribut::compute_normalsHough(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  //---------------------------

  int K = 100;
  int T = 1000;
  int n_phi = 15;
  int n_rot = 5;
  bool ua = false;
  float tol_angle_rad = 0.79;
  int k_density = 5;

  cout<<"Normals computation start ..."<<endl;
  double time_nrl;
  glfwSetTime(time_nrl);
  //-------------------------

  //Convert to Eigen cloud
  Eigen::MatrixX3d pc = Eigen::MatrixX3d::Zero(XYZ.size(),3);
  Eigen::MatrixX3d normals = Eigen::MatrixX3d::Zero(XYZ.size(),3);
  for(int i=0; i<XYZ.size(); i++){
    for(int j=0; j<3; j++){
      pc(i,j) = XYZ[i][j];
      normals(i,j) = Nxyz[i][j];
    }
  }

  //Compute normals
  Eigen_Normal_Estimator ne(pc,normals);
  ne.set_K(K);
  ne.set_T(T);
  ne.set_density_sensitive(ua);
  ne.set_n_phi(n_phi);
  ne.set_n_rot(n_rot);
  ne.set_tol_angle_rad(tol_angle_rad);
  ne.set_K_density(k_density);
  ne.estimate_normals();

  //Convert to glm cloud
  Nxyz.clear();
  for(int i=0; i<XYZ.size(); i++){
    Nxyz.push_back(vec3(normals(i,0), normals(i,1), normals(i,2)));
  }

  //---------------------------
  mesh->normal.Initial = Nxyz;
  mesh->normal.hasData = true;
  time_nrl = glfwGetTime();
  cout<<"Normals computation stop in "<<time_nrl<<" s"<<endl;
}
void Attribut::compute_normalReorientation(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  //---------------------------

  float dist_XYZ, dist_Nxyz;
  for(int i=0; i<XYZ.size(); i++){
    dist_XYZ = distance(XYZ[i], vec3(0,0,0));
    dist_Nxyz = distance(XYZ[i] + Nxyz[i], vec3(0,0,0));

    if(dist_Nxyz > dist_XYZ){
      for(int j=0; j<3; j++){
        Nxyz[i][j] = -Nxyz[i][j];
      }
    }
  }

  //---------------------------
}
void Attribut::compute_cosIt(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  vector<float>& cosIt = mesh->attribut.cosIt;
  vector<float>& It = mesh->attribut.It;
  vec3 root = mesh->location.root;
  //---------------------------

  //check data
  cosIt.clear(); It.clear();
  if(dist.size() == 0){
    compute_Distances(mesh);
  }
  if(Nxyz.size() == 0){
    cout<<"No normal for cosIt computation"<<endl;
    return;
  }

  //Compute cosIt
  for(int i=0; i<XYZ.size(); i++){
    float cIt = 0;

    //Compute cosIt
    for(int j=0; j<3; j++){
      cIt = cIt + ( -Nxyz[i][j] * ( (XYZ[i][j] - root[j]) / dist[i] ));
    }

    //Check for orientation
    if(cIt < 0){
      cIt = -cIt;
    }
    //Check for computability
    if(cIt >= 1){
      cIt = 0.9999;
    }

    cosIt.push_back(cIt);
    It.push_back( acos(cIt) * 180 / M_PI );
  }

  //---------------------------
}
void Attribut::compute_checkForNan(Mesh* mesh){
  vector<vec3>& Nxyz = mesh->normal.OBJ;
  vector<float>& cosIt = mesh->attribut.cosIt;
  vector<int> idx;
  //---------------------------

  //Compute cosIt
  for(int i=0; i<cosIt.size(); i++){
    if(isnan(Nxyz[i].x) == true){
      idx.push_back(i);
    }
    if(isnan(cosIt[i]) == true){
      idx.push_back(i);
    }
  }

  //---------------------------
  this->make_supressPoints(mesh, idx);
}

//Intensity
void Attribut::compute_intensityInversion(){
  if(sceneManager->is_atLeastOneMesh()){
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    if(mesh->intensity.hasData){
      vector<float>& Is = mesh->intensity.OBJ;

      for(int i=0; i<Is.size(); i++){
        Is[i] = 1 - Is[i];
      }
    }

    //---------------------------
  }
}
void Attribut::compute_colorToIntensity(Mesh* mesh){
  vector<float>& Is_obj = mesh->intensity.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  Is_obj.clear();
  //---------------------------

  if(mesh->color.hasData){
    for(int i=0; i<RGB.size(); i++){
      float I = (RGB[i].x + RGB[i].y + RGB[i].z) / 3;
      Is_obj.push_back(I);
    }
  }

  //---------------------------
  mesh->intensity.hasData = true;
}
void Attribut::fct_convert255to2048(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  //-------------------------

  for (int i=0; i<Is.size(); i++){
    Is[i] = (Is[i] * 255 + 2048) / 4096;
  }

  //-------------------------
  sceneManager->update_IntensityToColor(mesh);
  sceneManager->update_CloudColor(mesh);
}
void Attribut::fct_convert2048to255(Mesh* mesh){
  static bool I_2048 = false;
  vector<float>& Is = mesh->intensity.OBJ;
  //-------------------------

  for (int i=0; i<Is.size(); i++){
    Is[i] = (Is[i] * 4096 - 2048) / 255;
  }

  //-------------------------
  sceneManager->update_IntensityToColor(mesh);
  sceneManager->update_CloudColor(mesh);
}
void Attribut::fct_moins(){
  if(sceneManager->is_listMeshEmpty())return;
  Mesh* mesh = sceneManager->get_selectedMesh();
  vector<float>& Is = mesh->intensity.OBJ;
  vector<vec3>& XYZ = mesh->location.OBJ;
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<float>& cosIt = mesh->attribut.cosIt;
  //-------------------------

  cosIt.erase(cosIt.begin());
  cosIt.push_back(0.0f);

  //-------------------------
  sceneManager->update_IntensityToColor(mesh);
  sceneManager->update_CloudColor(mesh);
}
void Attribut::fct_IsRange(vec2 range){
  Mesh* mesh = sceneManager->get_selectedMesh();
  vector<float>& Is = mesh->intensity.OBJ;
  const vector<float>& Is_ini = mesh->intensity.Initial;
  vector<float>& Is_buff = mesh->intensity.Buffer;
  //---------------------------

  for(int i=0; i<Is.size(); i++){
    if(Is[i] < range.x || Is[i] > range.y){
      Is[i] = -1.0f;
    }else{
      Is[i] = Is_ini[i];
    }
  }

  //-------------------------
  sceneManager->update_IntensityToColor(mesh);
  sceneManager->update_CloudColor(mesh);
}
vec2 Attribut::get_IsRange(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  if(!sceneManager->is_listMeshEmpty() && Is.size() != 0){
    float min = Max(Is);
    for(int i=0; i<Is.size(); i++){
      if(min > Is[i] && Is[i] != -1.0f){
        min = Is[i];
      }
    }

    vec2 range = vec2(min, Max(Is));
    return range;
  }else{
    vec2 range = vec2(0, 0);
    return range;
  }

  //---------------------------
}
