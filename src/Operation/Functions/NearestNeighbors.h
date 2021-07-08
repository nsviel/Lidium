#ifndef NearestNeighbors_h
#define NearestNeighbors_h

#include "../../Parameters.h"

class NearestNeighbors
{
public:
  //Constructor / Destructor
  NearestNeighbors();
  ~NearestNeighbors();

public:
  MatrixXf KdTreeFLANN(vector<vec2> PT_trg, vector<vec2> PT_query, bool normalized);
  vector<int> kNN_number(vector<vec3>& PS, float pt_A, float pt_R);
  vector<int> kNN_radius(vector<vec3>& PS, float cosIt, float dist);
  int sequentialSearch(vector<vec3>& PS, float cosIt, float R);
  int binarySearch(vector<vec3>& PS, float cosIt, float R);
  void KdTreeNanoFLANN(Mesh* mesh_P, Mesh* mesh_Q);

private:

};

#endif
