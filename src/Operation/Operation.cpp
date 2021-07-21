#include "Operation.h"

#include "../Engine/Scene.h"
#include "../Engine/Glyphs.h"
#include "../Registration/Registration.h"
#include "Functions/Selection.h"
#include "../Load/Loader.h"

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/io.h>

//Constructor / destructor
Operation::Operation(Scene* scene, Glyphs* glyph){
  this->sceneManager = scene;
  this->glyphManager = glyph;
  //---------------------------

  //Get absolute executable location
  char path[PATH_MAX];
  char* osef = getcwd(path, sizeof(path));
  string abspath(path);
  this->pathDir = abspath + '/' + configuration.INIT_DefaultDirPath;

  this->spaceSampling = 0.08f;
  this->nbLineSampling = 1000000;

  //---------------------------
}
Operation::~Operation(){}

//Specific functions
void Operation::fastScene(int mode){
  char path[PATH_MAX];
  //---------------------------

  switch(mode){
    case 0:{//Buddha
      sceneManager->removeCloud_all();
      sceneManager->loadCloud("../media/fastScene/buddha.pts");
      Matrix4f realTransformation;
      realTransformation <<
        0.306093,   -0.951146,  -0.0403608,    0.585185,
        0.951862,      0.3065, -0.00415026 ,   0.289215,
        0.016318,  -0.0371476 ,   0.999177,   0.0189446,
        0,           0,           0,           1;
      sceneManager->loadCloud("../media/fastScene/buddha_moved.pts", realTransformation);
      break;
    }
    case 1:{//Torus
      sceneManager->removeCloud_all();
      sceneManager->loadCloud("../media/fastScene/torus_1.ply");
      sceneManager->loadCloud("../media/fastScene/torus_2.ply");
      break;
    }
    case 2:{//Spectralon
      sceneManager->removeCloud_all();
      sceneManager->loadCloud("../media/fastScene/cloud.pts");
      break;
    }

    /* Example :
    case 0:{
      sceneManager->removeCloud_all();
      Matrix4f realTransformation;
      realTransformation <<   // Real transformation for register the next point cloud to the second
      -0.2016,    0.9793,   -0.0148,   -5.3515,
      -0.9794,   -0.2017,   -0.0052,   -3.0974,
      -0.0081,    0.0134,    0.9999,    -0.18,
            0,         0,         0,    1.0000;
      sceneManager->loadCloud("../path_of_the_cloud_to_register", realTransformation);
      sceneManager->loadCloud("../path_of_the_cloud_statique");
      break;
    }
    */
  }

  //---------------------------
}
void Operation::reset(){
  list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
  Mesh* mesh = sceneManager->get_selectedMesh();
  //Fusion* fusionManager = rendererManager->get_FusionManager();
  //Selection* selectionManager = rendererManager->get_selectionManager();
  //---------------------------

  //Reset all clouds
  for(int i=0; i<list_Mesh->size(); i++){
    Mesh* mesh = *next(list_Mesh->begin(),i);
    sceneManager->update_ResetMesh(mesh);
  }

  //Reset all functions
  glyphManager->reset();
  //fusionManager->reset();
  sceneManager->update_allCloudData(mesh);

  //---------------------------
  //selectionManager->mark_supressAll();
  console.AddLog("Reset scene...");
}
void Operation::loading(){
  string zenity = "zenity --file-selection --multiple --title=Load --filename=" + pathDir + " 2> /dev/null";
  FILE *file = popen(zenity.c_str(), "r");
  char filename[32768];
  const char* path_char = fgets(filename, 32768, file);
  //---------------------------

  //Check if not empty
  if ((path_char != NULL) && (path_char[0] != '\0')){
    string path_str(path_char);

    //Check for multiple
    vector<string> CloudPaths;
    if (path_str.find('|')){
      int N = count(path_str.begin(), path_str.end(), '|');
      for(int i=0; i<N; i++){
        CloudPaths.push_back(path_str.substr(0, path_str.find('|')));
        path_str = path_str.substr(path_str.find('|')+1);
      }
    }
    CloudPaths.push_back(path_str);

    //Load files
    for(int i=0; i<CloudPaths.size(); i++){
      //Extract file path
      string path = CloudPaths[i];
      if (path_str.find('\n')){
        path.erase(std::remove(path.begin(), path.end(), '\n'), path.end()); //-> Supress unwanted line break
      }
      pathDir = path.substr(0, path.find_last_of("/")+1);

      //Load files
      sceneManager->loadCloud(path);
    }
  }

  //---------------------------
}
void Operation::loading(string folderPath){
  string zenity = "zenity --file-selection --multiple --title=Load --filename=" + folderPath + " 2> /dev/null";
  FILE *file = popen(zenity.c_str(), "r");
  char filename[32768];
  const char* path_char = fgets(filename, 32768, file);
  //---------------------------

  //Check if not empty
  if ((path_char != NULL) && (path_char[0] != '\0')){
    string path_str(path_char);

    //Check for multiple
    vector<string> CloudPaths;
    if (path_str.find('|')){
      int N = count(path_str.begin(), path_str.end(), '|');
      for(int i=0; i<N; i++){
        CloudPaths.push_back(path_str.substr(0, path_str.find('|')));
        path_str = path_str.substr(path_str.find('|')+1);
      }
    }
    CloudPaths.push_back(path_str);

    //Load files
    for(int i=0; i<CloudPaths.size(); i++){
      //Extract file path
      string path = CloudPaths[i];
      if (path_str.find('\n')){
        path.erase(std::remove(path.begin(), path.end(), '\n'), path.end()); //-> Supress unwanted line break
      }

      //Load files
      sceneManager->loadCloud(path);
    }
  }

  //---------------------------
}
void Operation::selectDirectory(string* folderPath){
  string zenity = "zenity --file-selection --directory --title=Directory --filename=" + *folderPath + " 2> /dev/null";
  FILE *file = popen(zenity.c_str(), "r");
  char filename[32768];
  const char* path_char = fgets(filename, 32768, file);
  //---------------------------

  //Check if not empty
  if ((path_char != NULL) && (path_char[0] != '\0')){
    string path_str(path_char);

    //Supress unwanted line break
    if (path_str.find('\n')){
      path_str.erase(std::remove(path_str.begin(), path_str.end(), '\n'), path_str.end());
    }

    //Change path dir
    *folderPath = path_str;
  }

  //---------------------------
}
void Operation::loading_sampling(){
  string zenity = "zenity --file-selection --multiple --title=Load --filename=" + pathDir + " 2> /dev/null";
  FILE *file = popen(zenity.c_str(), "r");
  char filename[32768];
  const char* path_char = fgets(filename, 32768, file);
  //---------------------------

  //Check if not empty
  if ((path_char != NULL) && (path_char[0] != '\0')){
    string path_str(path_char);

    //Check for multiple
    vector<string> CloudPaths;
    if (path_str.find('|')){
      int N = count(path_str.begin(), path_str.end(), '|');
      for(int i=0; i<N; i++){
        CloudPaths.push_back(path_str.substr(0, path_str.find('|')));
        path_str = path_str.substr(path_str.find('|')+1);
      }
    }
    CloudPaths.push_back(path_str);

    //Load files
    for(int i=0; i<CloudPaths.size(); i++){
      //Extract file path
      string path = CloudPaths[i];
      if (path_str.find('\n')){
        path.erase(std::remove(path.begin(), path.end(), '\n'), path.end()); //-> Supress unwanted line break
      }
      pathDir = path.substr(0, path.find_last_of("/")+1);

      //Load files
      int size = get_fileSize(path);
      if(size > 1000000){
        this->samplingLoader(path);
      }
    }
  }

  //---------------------------
}
void Operation::loading_treatment(){
  Loader loaderManager;
  string zenity = "zenity --file-selection --multiple --title=Load --filename=" + pathDir + " 2> /dev/null";
  FILE *file = popen(zenity.c_str(), "r");
  char filename[32768];
  const char* path_char = fgets(filename, 32768, file);
  //---------------------------

  //Check if not empty
  if ((path_char != NULL) && (path_char[0] != '\0')){
    string path_str(path_char);

    //Check for multiple
    vector<string> CloudPaths;
    if (path_str.find('|')){
      int N = count(path_str.begin(), path_str.end(), '|');
      for(int i=0; i<N; i++){
        CloudPaths.push_back(path_str.substr(0, path_str.find('|')));
        path_str = path_str.substr(path_str.find('|')+1);
      }
    }
    CloudPaths.push_back(path_str);

    //Load files
    for(int i=0; i<CloudPaths.size(); i++){
      //Extract file path
      string path = CloudPaths[i];
      if (path_str.find('\n')){
        path.erase(std::remove(path.begin(), path.end(), '\n'), path.end()); //-> Supress unwanted line break
      }
      pathDir = path.substr(0, path.find_last_of("/")+1);

      //Load files
      sceneManager->loadCloud(path);
      Mesh* mesh = sceneManager->get_selectedMesh();
      loaderManager.save_cloud(mesh, path);
      sceneManager->removeCloud(mesh);
    }
  }

  //---------------------------
}
void Operation::loadingFolder(string path){
  //---------------------------

  for(const auto& entry : std::experimental::filesystem::directory_iterator(path)){
    sceneManager->loadCloud(entry.path());
  }

  //---------------------------
}
void Operation::samplingLoader(string path){
/*  int nbLinePart = nbLineSampling;
  int nbPart = 0;
  int lmin = 0;
  int lmax = nbLinePart;
  int size = get_fileSize(path);
  Filter filterManager;
  //---------------------------

  //Load the "base" part
  sceneManager->loadCloud(path, lmin, lmax);
  Mesh* mesh_base = sceneManager->get_selectedMesh();
  Mesh* mesh_toAdd;

  //load and merge all other parts
  cout<<"Parameters : \n-> Nb lines per part : "<<nbLineSampling;
  cout<<"\n-> Space sampling : "<<spaceSampling<<endl;;
  while(lmax < size){
    cout<<"Loading part n°"<<nbPart;
    lmin = lmax;
    lmax = lmax + nbLinePart;
    nbPart++;

    //Load nth part
    sceneManager->loadCloud(path, lmin, lmax);
    mesh_toAdd = sceneManager->get_selectedMesh();

    //Merge the nth part with the base
    extractionManager->merging_addCloud(mesh_base, mesh_toAdd);

    //Supress nth added part
    sceneManager->removeCloud(mesh_toAdd);

    //Space sampling the resulting mesh
    filterManager.spaceSampling(mesh_base, spaceSampling);
    cout<<" - "<<mesh_base->NbPoints<<" points"<<endl;

    //Update final cloud
    sceneManager->update_allCloudData(mesh_base);
  }
*/
  //---------------------------
}
void Operation::saving(){
  Loader loaderManager;
  string zenity = "zenity --file-selection --save --title=Save --filename=" + pathDir;
  FILE *file = popen(zenity.c_str(), "r");
  char filename[1024];
  char* path_char = fgets(filename, 1024, file);
  //---------------------------

  //Check if empty
  if ((path_char != NULL) && (path_char[0] != '\0')) {
    string path_str(path_char);
    if (path_str.find('\n')){
      //Supress unwanted line break
      path_str.erase(std::remove(path_str.begin(), path_str.end(), '\n'), path_str.end());
    }

    Mesh* mesh = sceneManager->get_selectedMesh();
    loaderManager.save_cloud(mesh, path_str);
  }

  //---------------------------
}
void Operation::allSaving(){
  string zenity = "zenity --file-selection --directory --title=Save --filename=" + pathDir;
  FILE *file = popen(zenity.c_str(), "r");
  char filename[1024];
  char* path_char = fgets(filename, 1024, file);
  //---------------------------

  //Check if empty
  if ((path_char != NULL) && (path_char[0] != '\0')) {
    string path_str(path_char);
    if (path_str.find('\n'))
      path_str.erase(std::remove(path_str.begin(), path_str.end(), '\n'), path_str.end()); //-> Supress unwanted line break

    sceneManager->saveCloud_all(path_str);
  }

  //---------------------------
}
void Operation::detectSphere(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = glm_to_pcl_XYZ(mesh);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  vector<int> inliers;
  //---------------------------

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
  ransac.setDistanceThreshold (0.001);
  ransac.computeModel();
  ransac.getInliers(inliers);

  //---------------------------
  pcl::copyPointCloud (*cloud, inliers, *final);
  Mesh mesh_out = pcl_to_glm_XYZ(final);
  mesh->location.OBJ = mesh_out.location.OBJ;
  sceneManager->update_CloudPosition(mesh);
}
