#include "file_PLY.h"

//Constructor / Destructor
filePLY::filePLY(){}
filePLY::~filePLY(){}

//Main functions
bool filePLY::Loader(string pathFile){
  //---------------------------

  //Initialization
  this->Loader_init();

  //Retrieve data
  std::ifstream infile(pathFile);
  while (std::getline(infile, line))
  {
    cpt++;

    if(!endHeader){
      this->Loader_header();
    }
    else if(!endCheckProperties){
      this->Loader_property();
    }
    else if(!endData){
      this->Loader_data();
    }
  }

  if(blenderFile){
    this->Loader_DoubledPts();
  }

  //---------------------------
  return true;
}
bool filePLY::Exporter(string path, Mesh* mesh){
  //---------------------------

  //Create file
  if(path.substr(path.find_last_of(".") + 1) != "ply") path.append(".ply");
  ofstream file;
  file.open(path);
  if(!file)
  {
    cout<<"Error in creating file !"<<endl;
    return 0;
  }

  //-> Data : xyz (R) (rgb) (nxnynz)
  vector<vec3>& pos = mesh->location.Buffer;
  vector<vec4>& col = mesh->color.Buffer;
  vector<vec3>& nor = mesh->normal.Buffer;
  vector<float>& ref = mesh->intensity.Buffer;
  int precision = 6;

  //Write in the file
  //====== Header ========
  file << "ply" <<endl;
  file << "format ascii 1.0" <<endl;
  file << "element vertex " << pos.size() << endl;
  file << "property float x" << endl;
  file << "property float y" << endl;
  file << "property float z" << endl;
  if(mesh->color.hasData)
  {
      file << "property uchar red" << endl;
      file << "property uchar green" << endl;
      file << "property uchar blue" << endl;
  }
  if(mesh->normal.hasData)
  {
      file << "property float nx" << endl;
      file << "property float ny" << endl;
      file << "property float nz" << endl;
  }
  if(mesh->intensity.hasData)
  {
      file << "property float scalar_Scalar_field" << endl;
  }
  file << "element face 0" << endl;
  file << "property list uchar int vertex_indices" << endl;
  file << "end_header" <<endl;

  //====== Body ========
  for(int i=0; i<pos.size(); i++)
  {
    file << fixed;
    //---> xyz
    file << setprecision(precision) << pos[i].x <<" "<< pos[i].y <<" "<< pos[i].z ;

    //---> rgb
    //Color only
    if(mesh->color.hasData)
      file << setprecision(0) <<" "<< col[i].x * 255 <<" "<< col[i].y * 255 <<" "<< col[i].z * 255;

    //---> nx ny nz
    if(mesh->normal.hasData)
      file << setprecision(precision) <<" "<< nor[i].x <<" "<< nor[i].y <<" "<< nor[i].z;

    //---> R
    if(mesh->intensity.hasData)
      file << setprecision(0) <<" "<< (ref[i]*4096)-2048;

    //We end the line
    file << endl;
  }

  //---------------------------
  file.close();
  return true;
}

//Subfunctions
void filePLY::Loader_init(){
  //---------------------------

  this->locationOBJ.clear();
  this->normalOBJ.clear();
  this->colorOBJ.clear();
  this->intensityOBJ.clear();

  this->endHeader = false;
  this->endData = false;
  this->endProperties = false;
  this->endCheckProperties = false;
  this->RGBAlpha = false;
  this->blenderFile = false;

  this->config = 0;
  this->cpt = 0;
  this->colorColum = 0;
  this->normalColumn = 0;
  this->reflectanceColumn = 0;
  this->intensityColum = 0;
  this->positionColumn = 0;
  this->nbVertex = 0;

  //---------------------------
}
void filePLY::Loader_header(){
  string h1, h2, h3, h4;
  std::istringstream iss(line);
  //---------------------------

  iss >> h1 >> h2 >> h3 >> h4;

  if(h1 == "property" && h3 == "x") positionColumn = cpt;
  if(h1 == "property" && h3 == "red") colorColum = cpt;
  if(h1 == "property" && h3 == "nx") normalColumn = cpt;
  if(h1 == "property" && h3 == "intensity") intensityColum = cpt;
  if(h1 == "property" && h3 == "scalar_Scalar_field") reflectanceColumn = cpt;
  if(h1 == "property" && h3 == "alpha") RGBAlpha = true;
  if(h1 == "element" && h2 == "vertex") nbVertex = stoi(h3);
  if(h4 == "Blender") blenderFile = true;
  if(h1 == "end_header") endHeader = true;

  //---------------------------
}
void filePLY::Loader_property(){
  string dataFormat;
  //---------------------------

  //IF Colors + Normals + Reflectances
  if(colorColum !=0 && normalColumn !=0 && reflectanceColumn !=0){
    if(normalColumn < colorColum && normalColumn < reflectanceColumn && colorColum < reflectanceColumn){
      config = 1;
      dataFormat = "XYZ - Nxyz - RGB - I";
    }
    if(colorColum < normalColumn && normalColumn < reflectanceColumn && colorColum < reflectanceColumn){
      config = 2;
      dataFormat = "XYZ - RGB - Nxyz - I";
    }
  }
  //IF Colors + Normals
  if(colorColum !=0 && normalColumn !=0 && reflectanceColumn ==0){
    if(normalColumn < colorColum && RGBAlpha){
      config = 3;
      dataFormat = "XYZ - Nxyz - RGBA";
    }
    if(normalColumn > colorColum && !RGBAlpha){
      config = 8;
      dataFormat = "XYZ - RGB - Nxyz";
    }
  }
  //IF Normals only
  if(colorColum ==0 && normalColumn !=0 && reflectanceColumn ==0){
      config = 4;
      dataFormat = "XYZ - Nxyz";
  }
  //IF Colors only
  if(colorColum !=0 && normalColumn ==0 && reflectanceColumn ==0){
      config = 5;
      dataFormat = "XYZ - RGBA";
  }
  //IF Colors & reflectances
  if(colorColum !=0 && normalColumn ==0 && reflectanceColumn !=0){
      config = 6;
      dataFormat = "XYZ - RGB - I";
  }
  //IF nothing but anything else
  if(colorColum ==0 && normalColumn ==0 && reflectanceColumn ==0 && intensityColum !=0)
  {
      config = 7;
      dataFormat = "XYZ - I";
  }

  //---------------------------
  endCheckProperties = true;
}
void filePLY::Loader_data(){
  float x,y,z,r,g,b,nx,ny,nz,R,A,I;
  std::istringstream iss(line);
  //---------------------------

  switch(config){
    case 1: iss >> x>>y>>z >> nx>>ny>>nz>> r>>g>>b >> R; break;
    case 2: iss >> x>>y>>z >> r>>g>>b >> nx>>ny>>nz >> R; break;
    case 3: iss >> x>>y>>z >> nx>>ny>>nz>> r>>g>>b >> A; break;
    case 4: iss >> x>>y>>z >> nx>>ny>>nz; break;
    case 5: iss >> x>>y>>z >> r>>g>>b>>A; break;
    case 6: iss >> x>>y>>z >> r>>g>>b>>R; break;
    case 7: iss >> x>>y>>z >> I; break;
    case 8: iss >> x>>y>>z >> r>>g>>b >> nx>>ny>>nz; break;
    default : iss >> x>>y>>z; break;
  }

  //End of vertex data
  //cout<<"cpt : "<<cpt<<" nbvertex : "<<nbVertex-20<<endl;
  if((x == 4 || x == 3 || abs(x) < 0.00001) && cpt > nbVertex-20){
    endData = true;
    return;
  }

  //-------- Extract data --------
  //Position
  locationOBJ.push_back(vec3(x, y, z));

  //Normals
  if(normalColumn !=0)
    normalOBJ.push_back(vec3(nx, ny, nz));

  //Reflectances
  if(reflectanceColumn !=0)
    intensityOBJ.push_back((R+2048)/4096);

  //Intensity
  if(intensityColum !=0)
    intensityOBJ.push_back(I);

  //Colors
  if(colorColum !=0){
    colorOBJ.push_back(vec4( r/255, g/255, b/255, 1.0f));
  }
  else if(colorColum !=0 && RGBAlpha){
    colorOBJ.push_back(vec4( r/255, g/255, b/255, A/255));
  }

  //---------------------------
}
void filePLY::Loader_DoubledPts(){
  //---------------------------

  for(int i=0; i<locationOBJ.size(); i++){
    for(int j=0; j<locationOBJ.size(); j++){
      if(locationOBJ[i].x == locationOBJ[j].x &&
        locationOBJ[i].y  == locationOBJ[j].y &&
        locationOBJ[i].z == locationOBJ[i].z)
      {
        locationOBJ.erase(locationOBJ.begin() + j);
        if(colorOBJ.size() != 0) colorOBJ.erase(colorOBJ.begin() + j);
        if(normalOBJ.size() != 0) normalOBJ.erase(normalOBJ.begin() + j);
      }
    }
  }

  //---------------------------
}
