#ifndef SHADER_H
#define SHADER_H

#include "../../Parameters.h"

class Shader
{
public:
  Shader();
  ~Shader();

public:
  bool shaderManagment(string dir);
  bool detachShaders(GLuint shaderID);
  bool linkProgram();
  GLuint shader_compileVertex(string fileName);
  GLuint shader_compileFragment(string fileName);

  inline uint get_programID(){return programID;};
  inline uint get_modelID(){return modelID;};
  inline uint get_mvpID(){return mvpID;};
  inline uint get_comID(){return comID;};

private:
  uint programID;
  uint mvpID;
  uint comID;
  uint modelID;

  vector<int> list_Shader;
};

#endif
