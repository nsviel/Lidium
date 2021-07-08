#include "dataExtraction.h"

//Constructor / Destructor
dataExtraction::dataExtraction(){
  //---------------------------

  this->ID = 0;

  //---------------------------
}
dataExtraction::~dataExtraction(){}

//Main function
Mesh* dataExtraction::extractData(string filePath, vector<vec3>& locationOBJ, vector<float>& intensityOBJ, vector<vec4>& colorOBJ, vector<vec3>& normalOBJ){
  Mesh* mesh = new Mesh();
  //---------------------------

  //Init mesh parameters
  this->initCloudParameters(mesh, filePath);
  this->check_data(mesh, locationOBJ, intensityOBJ, colorOBJ, normalOBJ);

  //Data
  this->extract_Location(mesh, locationOBJ);
  this->extract_Intensity(mesh, intensityOBJ);
  this->extract_Color(mesh, colorOBJ);
  this->extract_Normal(mesh, normalOBJ);

  //Clear data
  locationOBJ.clear();
  normalOBJ.clear();
  colorOBJ.clear();
  intensityOBJ.clear();

  //---------------------------
  return mesh;
}

//Subfunctions
void dataExtraction::initCloudParameters(Mesh* mesh, string filePath){
  uint VAO;
  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);
  mesh->VAO = VAO;
  mesh->ID = ID;
  ID++;
  //---------------------------

  //Information
  string nameFormat = filePath.substr(filePath.find_last_of("/\\") + 1);
  mesh->Name =  nameFormat.substr(0, nameFormat.find_last_of("."));
  mesh->Format = nameFormat.substr(nameFormat.find_last_of("."), string::npos);
  mesh->pointSize = 1;

  //Parameters
  mesh->dataFormat = " ";
  mesh->location.root = vec3(0.0);
  mesh->color.hasData = false;
  mesh->normal.hasData = false;
  mesh->intensity.hasData = false;
  mesh->intensity.heatmap = false;
  mesh->intensity.corrected = false;
  mesh->intensity.linearized = false;
  mesh->visibility = true;

  //Transformation matrices
  mesh->transformation.Scale = mat4(1.0);
  mesh->transformation.Translation = mat4(1.0);
  mesh->transformation.Rotation = mat4(1.0);
  mesh->transformation.TransformationMatrix = mat4(1.0);
  mesh->transformation.RealTransformation = mat4(1.0);

  //---------------------------
}
void dataExtraction::extract_Location(Mesh* mesh, vector<vec3>& locationOBJ){
  uint positionVBO;
  //---------------------------

  glGenBuffers(1, &positionVBO);
  glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
  glBufferData(GL_ARRAY_BUFFER, locationOBJ.size()*sizeof(glm::vec3), &locationOBJ[0], GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  mesh->NbPoints = locationOBJ.size();
  mesh->location.VBO = positionVBO;
  mesh->location.Initial = locationOBJ;
  mesh->location.OBJ = locationOBJ;
  mesh->location.Buffer = locationOBJ;

  //---------------------------
}
void dataExtraction::extract_Normal(Mesh* mesh, vector<vec3>& normalOBJ){
  //---------------------------

  if(isNormal){
    uint normalsVBO;
    glGenBuffers(1, &normalsVBO);
    glBindBuffer(GL_ARRAY_BUFFER, normalsVBO);
    glBufferData(GL_ARRAY_BUFFER, normalOBJ.size()*sizeof(glm::vec3), &normalOBJ[0], GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)(4*sizeof(float)));
    glEnableVertexAttribArray(1);

    mesh->normal.hasData = true;
    mesh->normal.VBO = normalsVBO;
    mesh->normal.Initial = normalOBJ;
    mesh->normal.OBJ = normalOBJ;
  }

  //---------------------------
}
void dataExtraction::extract_Intensity(Mesh* mesh, vector<float>& intensityOBJ){
  //---------------------------

  if(isIntensity){
    mesh->intensity.hasData = true;
    mesh->intensity.Initial = intensityOBJ;
    mesh->intensity.OBJ = intensityOBJ;
  }

  //---------------------------
}
void dataExtraction::extract_Color(Mesh* mesh, vector<vec4>& colorOBJ){
  uint colorVBO;
  //---------------------------

  glGenBuffers(1, &colorVBO);
  glBindBuffer(GL_ARRAY_BUFFER, colorVBO);
  glBufferData(GL_ARRAY_BUFFER, colorOBJ.size()*sizeof(glm::vec4), &colorOBJ[0], GL_DYNAMIC_DRAW);
  glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
  glEnableVertexAttribArray(3);

  mesh->color.VBO = colorVBO;
  mesh->color.Initial = colorOBJ;
  mesh->color.Buffer = colorOBJ;
  mesh->color.OBJ = colorOBJ;

  //---------------------------
}
void dataExtraction::check_data(Mesh* mesh, vector<vec3>& locationOBJ, vector<float>& intensityOBJ, vector<vec4>& colorOBJ, vector<vec3>& normalOBJ){
  //---------------------------

  //Normals
  this->isNormal = false;
  if(normalOBJ.size() != 0 && normalOBJ.size() == locationOBJ.size()){
    this->isNormal = true;
  }

  //Intensities
  this->isIntensity = false;
  if(intensityOBJ.size() != 0 && intensityOBJ.size() == locationOBJ.size()){
    this->isIntensity = true;
  }

  //Colors
  //---> Compute a random color for each cloud
  float Red, Green, Blue;
  Red = float(rand()%101)/100;
  Green = float(rand()%101)/100;
  Blue = float(rand()%101)/100;
  mesh->color.uniColor = vec3(Red, Green, Blue);

  //---> if color data
  if(colorOBJ.size() != 0){
    mesh->color.hasData = true;
  }
  //---> if intensity data
  else if(colorOBJ.size() == 0 && intensityOBJ.size() != 0){
    for(int i=0; i<intensityOBJ.size(); i++){
      colorOBJ.push_back(vec4(intensityOBJ.at(i), intensityOBJ.at(i), intensityOBJ.at(i), 1.0f));
    }
  }
  //---> if no color or intensity data
  else{
    for(int i=0; i<locationOBJ.size(); i++){
      colorOBJ.push_back(vec4(Red, Green, Blue, 1.0f));
    }
  }

  //---------------------------
}
