#ifndef ICP_REJECTION_H
#define ICP_REJECTION_H

#include "../../Parameters.h"

class ICP_Rejection
{
public:
  //Constructor / Destructor
  ICP_Rejection();
  ~ICP_Rejection();

public:
  //ICCP implementation
  void weighting_ICCP(Mesh* mesh_P, Mesh* mesh_Q);

  //Rejection methods
  void rejection_distance(Mesh* mesh_P, Mesh* mesh_Q, float threshold);
  void rejection_color(Mesh* mesh_P, Mesh* mesh_Q);
  void rejection_normal(Mesh* mesh_P, Mesh* mesh_Q);
  void rejection_duplicata(Mesh* mesh_P, Mesh* mesh_Q);

  //Subfunctions
  void make_supressPoints(vector<vec3>& XYZ, vector<int>& idx);

private:
  float thr_color;
};

#endif
