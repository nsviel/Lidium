#include "Registration.h"

#include "Algo_PCL.h"
#include "Pipeline/ICP.h"
#include "Pipeline/ICP_Matching.h"
#include "../Operation/Transforms.h"
#include "../Engine/Glyphs.h"
#include "../Engine/Scene.h"
#include "../Operation/Attribut.h"
#include "../Operation/Plotting.h"

//Constructor / Destructor
Registration::Registration(Scene* scene, Glyphs* glyph){
  //---------------------------

  this->attribManager = new Attribut(sceneManager);
  this->sceneManager = scene;
  this->glyphManager = glyph;
  this->icpManager = new ICP(glyphManager);
  this->pclManager = new Algo_PCL();
  this->plotManager = new Plotting();

  this->SSE = 10;
  this->SSE_max = 0.000;
  this->nbIter = 0;
  this->nbIter_max = 2500;
  this->colorMeth = 1;

  //---------------------------
}
Registration::~Registration(){}

//Main functions
void Registration::make_Iteration(){
  Mesh* mesh_P = sceneManager->get_selectedMesh();
  Mesh* mesh_Q = sceneManager->get_otherMesh();
  //------------------------------

  //Cloud coloration
  this->colorization(mesh_P, mesh_Q);

  //compute SSE
  SSE = icpManager->compute_SSE(mesh_P, mesh_Q);

  //If ending conditions are fulfill
  if(nbIter < nbIter_max && SSE > SSE_max){
    //ICP algorithm
    tic();
    icpManager->algo(mesh_P, mesh_Q);
    sceneManager->update_CloudPosition(mesh_P);
    duration = toc();

    //ICP results
    nbIter++;
    SSE = icpManager->compute_SSE_groundTruth(mesh_P);
    SSE_mean = icpManager->get_SSE_mean();
    vec_SSE.push_back(SSE);
    vec_iter.push_back(nbIter);
  }

  //------------------------------
  glyphManager->obj_matching(mesh_P, mesh_Q);
}
void Registration::make_algoPCL(int method){
  Mesh* mesh_P = sceneManager->get_selectedMesh();
  Mesh* mesh_Q = sceneManager->get_otherMesh();
  //------------------------------

  //Cloud coloration
  this->colorization(mesh_P, mesh_Q);

  //compute SSE
  SSE = icpManager->compute_SSE(mesh_P, mesh_Q);

  //If ending conditions are fulfill
  if(nbIter < nbIter_max && SSE > SSE_max){
    //PCL algorithms
    tic();
    switch(method){
      case 0:{ //ICP
        SSE = pclManager->algo_ICP(mesh_P, mesh_Q);
        break;
      }
      case 1:{ //NDT
        SSE = pclManager->algo_NDT(mesh_P, mesh_Q);
        break;
      }
      case 2:{ //GICP
        SSE = pclManager->algo_GICP(mesh_P, mesh_Q);
        break;
      }
      case 3:{ //LUM
        SSE = pclManager->algo_LUM(mesh_P, mesh_Q);
        break;
      }
      case 4:{ //4 points congruents
        SSE = pclManager->algo_4ptsCong(mesh_P, mesh_Q);
        break;
      }
    }

    //Update cloud position
    sceneManager->update_CloudPosition(mesh_P);
    duration = toc();
  }

  //------------------------------
}

//Subfunctions
void Registration::reset(){
  //---------------------------

  this->restart();
  icpManager->reset();

  //---------------------------
}
void Registration::restart(){
  //---------------------------

  this->nbIter = 0;
  this->SSE = 10;
  this->duration = 0;

  vec_SSE.clear();
  vec_iter.clear();
  vec_trans.clear();
  vec_rotat.clear();

  //---------------------------
}
void Registration::print_ICP(){
  if(vec_iter.size() < 2){
    cout<<"No enough data for plotting"<<endl;
    return;
  }
  //------------------------------

  plotManager->plot_Curve_discret(vec_iter, vec_SSE);

  //------------------------------
}
void Registration::colorization(Mesh* mesh_P, Mesh* mesh_Q){
  //---------------------------

  switch(colorMeth){
    case 0:{ //None
      if(mesh_P->color.hasData && mesh_Q->color.hasData){
        attribManager->set_colorRGB(mesh_P);
        attribManager->set_colorRGB(mesh_Q);
      }else if(mesh_P->intensity.hasData && mesh_Q->intensity.hasData){
        attribManager->set_colorI(mesh_P);
        attribManager->set_colorI(mesh_Q);
      }
      break;
    }
    case 1:{ //Reg / Green
      attribManager->set_pointCloudColor(mesh_P, vec4(1.0f,0.0f,0.0f,1.0f));
      attribManager->set_pointCloudColor(mesh_Q, vec4(0.0f,1.0f,0.0f,1.0f));
      break;
    }
    case 2:{ //Reg / Green -
      attribManager->set_pointCloudColor(mesh_P, vec4(1.0f,0.0f,0.0f,0.3f));
      attribManager->set_pointCloudColor(mesh_Q, vec4(0.0f,1.0f,0.0f,0.3f));
      break;
    }
    case 3:{ //Transparent
      sceneManager->set_MeshVisibility(mesh_P, false);
      sceneManager->set_MeshVisibility(mesh_Q, false);
      break;
    }
  }

  //---------------------------
}
