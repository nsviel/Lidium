#include "Loader.h"

#include <pcl/io/pcd_io.h>

//Constructor / Destructor
Loader::Loader(){}
Loader::~Loader(){}

//Loading function
bool Loader::load_cloud(string filePath){
  //---------------------------

  //Check file existence
  if(is_file_exist(filePath) == false){
    console.AddLog("[error] File doesn't exists: %s", filePath.c_str());
    return false;
  }

  //Check file format & retrieve data
  string format = filePath.substr(filePath.find_last_of(".") + 1);
  bool sucess;
  if     (format == "pts"){
    //Load file
    sucess = ptsManager.Loader(filePath);

    //Retrieve data
    locationOBJ = ptsManager.get_locationOBJ();
    intensityOBJ = ptsManager.get_intensityOBJ();
    normalOBJ = ptsManager.get_normalOBJ();
    colorOBJ = ptsManager.get_colorOBJ();
    ptsManager.Loader_clearData();
  }
  else if(format == "ptx"){
    //Load file
    sucess = ptxManager.Loader(filePath);

    //Retrieve data
    locationOBJ = ptxManager.get_locationOBJ();
    intensityOBJ = ptxManager.get_intensityOBJ();
    normalOBJ = ptxManager.get_normalOBJ();
    colorOBJ = ptxManager.get_colorOBJ();
  }
  else if(format == "pcd"){
    //Load file
    sucess = pcdManager.Loader(filePath);

    //Retrieve data
    locationOBJ = pcdManager.get_locationOBJ();
    intensityOBJ = pcdManager.get_intensityOBJ();
    normalOBJ = pcdManager.get_normalOBJ();
    colorOBJ = pcdManager.get_colorOBJ();
  }
  else if(format == "ply"){
    //Load file
    sucess = plyManager.Loader(filePath);

    //Retrieve data
    locationOBJ = plyManager.get_locationOBJ();
    intensityOBJ = plyManager.get_intensityOBJ();
    normalOBJ = plyManager.get_normalOBJ();
    colorOBJ = plyManager.get_colorOBJ();
  }
  else if(format == "obj"){
    sucess = OBJLoader(filePath);
  }
  else if(format == "pcd"){
    sucess = PCDLoader(filePath);
  }
  else{
    sucess = false;
  }

  //Check sucess
  if(!sucess || locationOBJ.size() == 0){
    console.AddLog("[error] Failing loading point cloud");
    return false;
  }

  //Extraction of data
  mesh = extractManager.extractData(filePath, locationOBJ, intensityOBJ, colorOBJ, normalOBJ);

  //---------------------------
  console.AddLog("[sucess] Loaded %s", filePath.c_str());
  return true;
}
bool Loader::load_cloud_silent(string filePath){
  //---------------------------

  //Check file existence
  if(is_file_exist(filePath) == false){
    console.AddLog("[error] File doesn't exists");
    return false;
  }

  //Check file format
  string format = filePath.substr(filePath.find_last_of(".") + 1);
  bool sucess;
  if     (format == "pts"){
    sucess = ptsManager.Loader(filePath);
    locationOBJ = ptsManager.get_locationOBJ();
    intensityOBJ = ptsManager.get_intensityOBJ();
    normalOBJ = ptsManager.get_normalOBJ();
    colorOBJ = ptsManager.get_colorOBJ();
    ptsManager.Loader_clearData();
  }
  else if(format == "ptx"){
    sucess = ptxManager.Loader(filePath);
    locationOBJ = ptxManager.get_locationOBJ();
    intensityOBJ = ptxManager.get_intensityOBJ();
    normalOBJ = ptxManager.get_normalOBJ();
    colorOBJ = ptxManager.get_colorOBJ();
  }
  else if(format == "ply"){
    sucess = plyManager.Loader(filePath);
    locationOBJ = plyManager.get_locationOBJ();
    intensityOBJ = plyManager.get_intensityOBJ();
    normalOBJ = plyManager.get_normalOBJ();
    colorOBJ = plyManager.get_colorOBJ();
  }
  else if(format == "obj"){
    sucess = OBJLoader(filePath);
  }
  else if(format == "pcd"){
    sucess = PCDLoader(filePath);
  }
  else{
    sucess = false;
  }

  if(!sucess){
    console.AddLog("[error] Failing loading point cloud");
    return false;
  }

  mesh = extractManager.extractData(filePath, locationOBJ, intensityOBJ, colorOBJ, normalOBJ);

  //---------------------------
  return true;
}
bool Loader::load_cloud_part(string filePath, int lmin, int lmax){
  //---------------------------

  //Check file existence
  if(is_file_exist(filePath) == false){
    console.AddLog("[error] File doesn't exists");
    return false;
  }

  //Check file format
  string format = filePath.substr(filePath.find_last_of(".") + 1);
  bool sucess;
  if(format == "pts"){
    sucess = ptsManager.Loader_subPart(filePath, lmin, lmax);
    locationOBJ = ptsManager.get_locationOBJ();
    intensityOBJ = ptsManager.get_intensityOBJ();
    normalOBJ = ptsManager.get_normalOBJ();
    colorOBJ = ptsManager.get_colorOBJ();
    ptsManager.Loader_clearData();
  }
  else{
    sucess = false;
  }


  if(!sucess){
    console.AddLog("[error] Failing loading point cloud");
    return false;
  }

  mesh = extractManager.extractData(filePath, locationOBJ, intensityOBJ, colorOBJ, normalOBJ);

  //---------------------------
  return true;
}
bool Loader::load_cloud_creation(Mesh* mesh_in){
  string filePath = mesh_in->Name + mesh_in->Format;
  //---------------------------

  //Clear data
  this->locationOBJ.clear();
  this->normalOBJ.clear();
  this->colorOBJ.clear();
  this->intensityOBJ.clear();

  //Take input data
  if(mesh_in->location.OBJ.size() != 0){
    locationOBJ = mesh_in->location.OBJ;
  }else{
    cout<<"No mesh points"<<endl;
  }
  if(mesh_in->color.hasData){
    colorOBJ = mesh_in->color.OBJ;

    if(mesh_in->color.OBJ.size() == 0){
      cout<<"No mesh color"<<endl;
    }
  }
  if(mesh_in->intensity.hasData){
    intensityOBJ = mesh_in->intensity.OBJ;

    if(mesh_in->intensity.OBJ.size() == 0){
      cout<<"No mesh intensity"<<endl;
    }
  }
  if(mesh_in->normal.hasData){
    normalOBJ = mesh_in->normal.OBJ;

    if(mesh_in->normal.OBJ.size() == 0){
      cout<<"No mesh normal"<<endl;
    }
  }

  //Extract data and return the resulting slice
  mesh = extractManager.extractData(filePath, locationOBJ, intensityOBJ, colorOBJ, normalOBJ);

  //---------------------------
  return true;
}
bool Loader::load_glyph(string filePath){
  //---------------------------

  //Load data
  ptsManager.Loader(filePath);
  locationOBJ = ptsManager.get_locationOBJ();
  colorOBJ = ptsManager.get_colorOBJ();
  ptsManager.Loader_clearData();

  //Extract data
  glyph = new Glyph();

  uint VAO, colorVBO, locationVBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &locationVBO);
  glGenBuffers(1, &colorVBO);
  glyph->VAO = VAO;
  glyph->VBO_location = locationVBO;
  glyph->VBO_color = colorVBO;
  glyph->location = locationOBJ;
  glyph->color = colorOBJ;

  string nameFormat = filePath.substr(filePath.find_last_of("/\\") + 1);
  glyph->Name =  nameFormat.substr(0, nameFormat.find_last_of("."));
  glyph->draw_type = "point";
  glyph->draw_width = 1;
  glyph->visibility = true;

  //Clear data
  vector<vec3>().swap(locationOBJ);
  vector<vec3>().swap(normalOBJ);
  vector<vec4>().swap(colorOBJ);
  vector<float>().swap(intensityOBJ);

  //---------------------------
  return true;
}
bool Loader::load_binary(){
  ifstream file ("../../example.bin", ios::in|ios::binary|ios::ate);
  //---------------------------

  if(file.is_open()){
    streampos size = file.tellg();
    char* buffer = new char [size];

    file.seekg(0, ios::beg);
    file.read(buffer, size);
    file.close();

    cout << "Open binary file... Ok"<<endl;
    std::cout.write(buffer, size)<<endl;

    delete[] buffer;
  }else{
    cout << "Problem openning binary file"<<endl;
    return false;
  }

  //---------------------------
  return true;
}
void Loader::load_option(int format, int option, bool value){
  //---------------------------

  switch(format){
    case 0:{//For all formats
      if(option == 0){//Intensity ON
        ptxManager.set_retrievingIntensity(value);
        ptsManager.set_retrievingIntensity(value);
      }else
      if(option == 1){//Color ON
        ptxManager.set_retrievingColor(value);
        ptsManager.set_retrievingColor(value);
      }else
      if(option == 2){//Normal ON
        ptxManager.set_retrievingNormal(value);
        ptsManager.set_retrievingNormal(value);
      }else
      if(option == 3){//I format 01
        ptxManager.set_IdataFormat(0);
        ptsManager.set_IdataFormat(0);
      }else
      if(option == 4){//I format 255
        ptxManager.set_IdataFormat(1);
        ptsManager.set_IdataFormat(1);
      }else
      if(option == 5){//I format 4096
        ptxManager.set_IdataFormat(2);
        ptsManager.set_IdataFormat(2);
      }
      break;
    }

    case 1:{//PTS options
      break;
    }

    case 2:{//PTX options
      if(option == 0){
        ptxManager.set_scannerAtOrigin(value);
      }else
      if(option == 1){
        ptxManager.set_applyCloudTransfo(value);
      }else
      if(option == 2){
        ptxManager.set_separateCloud(value);
      }
      if(option == 3){
        ptxManager.set_notUseZValue(value);
      }
      break;
    }
  }

  //---------------------------
}

//Saving function
bool Loader::save_cloud(Mesh* mesh, string filePath){
  string format = filePath.substr(filePath.find_last_of(".") + 1);
  bool sucess = false;
  //---------------------------

  //Check file format
  if(format.at(0) == '/') format = "pts";

  if     (format == "pts"){
    sucess = ptsManager.Exporter(filePath, mesh);
  }
  else if(format == "ply"){
    sucess = plyManager.Exporter(filePath, mesh);
  }

  //Say if save is successfull
  if(!sucess){
    console.AddLog("[error] Failing saving point cloud");
    return false;
  }

  //---------------------------
  console.AddLog("[sucess] Saved %s", filePath.c_str());
  return true;
}

//Specific formats
bool Loader::OBJLoader(string filePath){
  std::ifstream infile(filePath);
  std::string line;
  float a, b, c;
  string ID;
  //---------------------------

  float R = float(rand()%101)/100;
  float G = float(rand()%101)/100;
  float B = float(rand()%101)/100;

  while (std::getline(infile, line)){
    std::istringstream iss(line);
    iss >> ID >> a >> b >> c;

    //Data extraction
    if(ID == "v" ) locationOBJ.push_back(vec3(a, b, c));
    if(ID == "vn") normalOBJ.push_back(vec3(a, b, c));
  }

  //---------------------------
  return true;
}
bool Loader::PCDLoader(string filePath){
  //---------------------------

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  bool sucess = pcl::io::loadPCDFile <pcl::PointXYZRGB>(filePath, *cloud);

  this->locationOBJ = pcl_XYZRGB_to_glm_vecXYZ(cloud);
  this->colorOBJ = pcl_XYZRGB_to_glm_vecRGB(cloud);

  if(sucess == -1){
    std::cout << "Cloud reading failed." << std::endl;
    return false;
  }

  //---------------------------
  return true;
}
