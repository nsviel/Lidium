#include "file_PTX.h"

//Constructor / Destructor
filePTX::filePTX(){
  //---------------------------

  this->option_separateCloud = false;
  this->option_scannerAtOrigin = false;
  this->option_applyCloudTransfo = true;
  this->retrieve_I = true;
  this->retrieve_RGB = true;
  this->retrieve_N = true;
  this->IdataFormat = 0;

  //---------------------------
}
filePTX::~filePTX(){}

//Main functions
bool filePTX::Loader(string pathFile){
  this->locationOBJ.clear();
  this->colorOBJ.clear();
  this->intensityOBJ.clear();

  std::ifstream infile(pathFile);
  std::string line;
  list_ptxCloud = new list<PTXCloud*>;
  PTXCloud* mesh = new PTXCloud;
  PC_line = 0;
  //---------------------------

  //Data loop
  while (std::getline(infile, line)){
    std::istringstream iss(line);
    x = 0; y = 0; z = 0; I = 0; r = 0; g = 0; b = 0;
    iss >> x >> y >> z >> I >> r >> g >> b;

    //New header detected -> new cloud
    if(abs(y) == 0 && abs(z) == 0 && abs(I) == 0 && PC_line >= 10){
      if(option_separateCloud){
        break;
      }
      cout<<"New cloud - nb of lines : "<<PC_line<<endl;
      list_ptxCloud->push_back(mesh);
      mesh = new PTXCloud;
      PC_line = 0;
    }

    this->Loader_header(mesh);
    this->Loader_data(mesh);

    PC_line++;
  }
  list_ptxCloud->push_back(mesh);

  //Scanner at origin
  if(option_scannerAtOrigin){
    this->Loader_scannerAtOrigin();
  }

  //Apply transfomation
  if(option_applyCloudTransfo){
    this->Loader_cloudTransformation();
  }

  //Assemble all clouds
  this->Loader_assembling();

  //---------------------------
  return true;
}
bool filePTX::Exporter(string path){
  //---------------------------

  //Create file
  if(path.substr(path.find_last_of(".") + 1) != "ptx") path.append(".ptx");
  ofstream file;
  file.open(path);
  if(!file)
  {
    cout<<"Error in creating file !";
    return 0;
  }
  /*
  for(int i=0; i<list_Mesh->size(); i++)
  {
    //Select clouds one by one
    Mesh* mesh = &*next(list_Mesh->begin(),i);

    //----->HEADER
    //***********************************
    int nbRows = 3;
    if(mesh->intensity.hasData) nbRows++;
    if(mesh->color.hasData) nbRows++;
    if(mesh->normal.hasData) nbRows++;
    //number of columns
    file << mesh->NbPoints << endl;
    //number of rows
    file << nbRows << endl;
    //scanner registered position
    vec3& ScanPos = mesh->scanner.Position;
    file << ScanPos.x << " " << ScanPos.y << " " << ScanPos.z << endl;
    //scanner orientation
    file << 1 << " " << 0 << " " << 0 << endl;
    file << 0 << " " << 1 << " " << 0 << endl;
    file << 0 << " " << 0 << " " << 1 << endl;
    //transformation matrix
    mat4& transMat = mesh->transformation.Translation;
    mat4& rotMat = mesh->transformation.Rotation;
    mat4 finalMat = transpose(transMat * rotMat);
    file << setprecision(6) << finalMat[0][0] << " " << finalMat[0][1] << " " << finalMat[0][2] << " " << finalMat[0][3] << endl;
    file << setprecision(6) << finalMat[1][0] << " " << finalMat[1][1] << " " << finalMat[1][2] << " " << finalMat[1][3] << endl;
    file << setprecision(6) << finalMat[2][0] << " " << finalMat[2][1] << " " << finalMat[2][2] << " " << finalMat[2][3] << endl;
    file << setprecision(6) << finalMat[3][0] << " " << finalMat[3][1] << " " << finalMat[3][2] << " " << finalMat[3][3] << endl;
    //***********************************

    //----->DATA
    //-> Data : xyz (R) (rgb) (nxnynz)
    vector<vec3>& pos = mesh->location.Buffer;
    vector<vec4>& col = mesh->color.Buffer;
    vector<vec3>& nor = mesh->normal.Buffer;
    vector<float>& ref = mesh->intensity.Buffer;
    int precision = 6;

    //Write in the file
    file << pos.size() <<endl;
    for(int i=0; i<pos.size(); i++)
    {
      file << fixed;
      //---> xyz
      file << setprecision(precision) << pos[i].x <<" "<< pos[i].y <<" "<< pos[i].z ;

      //---> R
      if(mesh->intensity.hasData)
        file << setprecision(0) <<" "<< ref[i];

      //---> rgb
      //Color only
      if(mesh->color.hasData)
        file << setprecision(0) <<" "<< col[i].x * 255 <<" "<< col[i].y * 255 <<" "<< col[i].z * 255;

      //---> nx ny nz
      if(mesh->normal.hasData)
        file << setprecision(precision) <<" "<< nor[i].x <<" "<< nor[i].y <<" "<< nor[i].z;

      //We end the line
      file << endl;
    }
  }
  */

  //---------------------------
  file.close();
  return true;
}

//Subfunctions
void filePTX::Loader_header(PTXCloud* mesh){
  //---------------------------

  //Scanner registered position
  if(PC_line == 2){
    mesh->rootTrans.x = x;
    mesh->rootTrans.y = y;
    mesh->rootTrans.z = z;
  }
  //Scanner registered axis
  if(PC_line >= 3 && PC_line <= 5){
    mesh->rootRotat[PC_line-3].x = x;
    mesh->rootRotat[PC_line-3].y = y;
    mesh->rootRotat[PC_line-3].z = z;
  }
  //Transformation matrix
  if(PC_line >= 6 && PC_line <= 9){
    mesh->transfMat[PC_line-6].x = x;
    mesh->transfMat[PC_line-6].y = y;
    mesh->transfMat[PC_line-6].z = z;
    mesh->transfMat[PC_line-6].w = I;
  }

  //---------------------------
}
void filePTX::Loader_data(PTXCloud* mesh){
  //---------------------------

  if(PC_line > 9){
    if(abs(r) >= 0.0001){
      //Location
      mesh->location.push_back(vec3(x, y, z));

      //Intensity
      if(retrieve_I){
        if(IdataFormat == 0){
          mesh->intensity.push_back(I);
        }else
        if(IdataFormat == 1){
          mesh->intensity.push_back(I/255);
        }else
        if(IdataFormat == 2){
          mesh->intensity.push_back((I+2048)/4096);
        }
      }

      //Color
      if(retrieve_RGB){
        mesh->color.push_back(glm::vec4(r/255, g/255, b/255, 1.0f));
      }
    }
  }

  //---------------------------
}
void filePTX::Loader_assembling(){
  //Assemble into an unique cloud
  //---------------------------

  for(int i=0; i<list_ptxCloud->size(); i++){
    PTXCloud* mesh = *next(list_ptxCloud->begin(),i);

    for(int j=0; j<mesh->location.size(); j++){
      locationOBJ.push_back(mesh->location[j]);
      intensityOBJ.push_back(mesh->intensity[j]);
      colorOBJ.push_back(mesh->color[j]);
    }
  }
  //---------------------------
}
void filePTX::Loader_cloudTransformation(){
  cout<<"---> Apply ptx cloud transformation"<<endl;
  //---------------------------

  for(int i=0; i<list_ptxCloud->size(); i++){
    PTXCloud* mesh = *next(list_ptxCloud->begin(),i);
    vector<vec3>& XYZ = mesh->location;
    mat4& MatT = mesh->transfMat;

    if(option_notUseZValue){
      MatT[3][2] = 0;
    }

    for(int j=0; j<XYZ.size(); j++){
      vec4 pos = vec4(XYZ[j].x, XYZ[j].y, XYZ[j].z, 1.0) * transpose(MatT);
      XYZ[j] = vec3(pos.x, pos.y, pos.z);
    }
  }

  //---------------------------
}
void filePTX::Loader_scannerAtOrigin(){
  cout<<"---> Set cloud origin at scanner position"<<endl;
  //---------------------------

  for(int i=0; i<list_ptxCloud->size(); i++){
    PTXCloud* mesh = *next(list_ptxCloud->begin(),i);

    vector<vec3>& XYZ = mesh->location;
    vec3& scanTranslation = mesh->rootTrans;
    mat3& scanRotation = mesh->rootRotat;

    if(option_notUseZValue){
      scanTranslation.z = 0;
    }

    for(int j=0; j<XYZ.size(); j++){
      //Translation
      float x = XYZ[j].x + scanTranslation.x;
      float y = XYZ[j].y + scanTranslation.y;
      float z = XYZ[j].z + scanTranslation.z;
      XYZ[j] = vec3(x, y, z);

      //Rotation
      XYZ[j] = XYZ[j] * scanRotation;
    }
  }

  //---------------------------
}
