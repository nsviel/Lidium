#ifndef ALGO_PCL_H
#define ALGO_PCL_H

#include "../Parameters.h"

class Algo_PCL
{
public:
  //Constructor / Destructor
  Algo_PCL();
  ~Algo_PCL();

public:
  float algo_ICP(Mesh* mesh_P, Mesh* mesh_Q);
  float algo_NDT(Mesh* mesh_P, Mesh* mesh_Q);
  float algo_GICP(Mesh* mesh_P, Mesh* mesh_Q);
  float algo_LUM(Mesh* mesh_P, Mesh* mesh_Q);
  float algo_4ptsCong(Mesh* mesh_P, Mesh* mesh_Q);

  Matrix4f optimization_SVD(Mesh* mesh_1, Mesh* mesh_2);
  Matrix4f optimization_LM(Mesh* mesh_1, Mesh* mesh_2);
  Matrix4f optimization_DualQuaternion(Mesh* mesh_1, Mesh* mesh_2);

private:

};

#endif
