#ifndef ICP_MATCHING_H
#define ICP_MATCHING_H

#include "../../Parameters.h"

class ICP_Matching
{
public:
  //Constructor / Destructor
  ICP_Matching();
  ~ICP_Matching();

public:
  //Simple matching
  void algo_directMatching(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_userSelection(Mesh* mesh_P, Mesh* mesh_Q);

  //Geometric based
  void algo_NN_BruteForce_ICCP(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NN_BruteForce(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NN_OctreePCL(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NN_KdTreePCL(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NN_KdTreeFLANN(Mesh* mesh_P, Mesh* mesh_Q);

  //Intensity based
  void algo_NI_BruteForce(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NI_NN(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NI_NN_KdTreeFLANN(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NI_KdTreeFLANN(Mesh* mesh_P, Mesh* mesh_Q);
  void algo_NI_NN_KdTreeNanoFLANN(Mesh* mesh_P, Mesh* mesh_Q);

  inline vector<Uplet> get_idx(){return idx;}

  void algo_PFH(Mesh* mesh_P, Mesh* mesh_Q);

private:
  Mesh* mesh_P;
  Mesh* mesh_Q;
  vector<Uplet> idx;
};

#endif
