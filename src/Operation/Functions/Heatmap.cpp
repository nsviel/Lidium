#include "Heatmap.h"

#include "../../Engine/Scene.h"
#include "../Attribut.h"

//Constructor / destructor
HeatMap::HeatMap(Scene* scene){
  this->sceneManager = scene;
  //---------------------------

  this->attribManager = new Attribut(sceneManager);

  this->HMmode = 0;
  this->normalized = false;

  //---------------------------
}
HeatMap::~HeatMap(){}

//HMI functions
void HeatMap::set_HeatMap_all(bool heatAll){
  list<Mesh*>* list = sceneManager->get_listMesh();
  //---------------------------

  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);

    if(heatAll){
      mesh->intensity.heatmap = false;
    }else{
      mesh->intensity.heatmap = true;
    }

    this->set_HeatMap(mesh);
  }

  //---------------------------
}
void HeatMap::set_HeatMap(Mesh* mesh){
  bool& heatmap = mesh->intensity.heatmap;
  //---------------------------

  //Apply heatmap
  if(heatmap == false && mesh->intensity.hasData){
    this->compute_applyHeatmap(mesh);
  }

  //Reverse heatmap
  if(heatmap == true && mesh->intensity.hasData){
    this->compute_reverseHeatmap(mesh);
  }

  //---------------------------
  mesh->color.Buffer = mesh->color.OBJ;
  sceneManager->update_CloudColor(mesh);
  heatmap = !heatmap;
}
void HeatMap::set_HeatMapField(int value){
  this->HMmode = value;
  //---------------------------

  if(sceneManager->is_listMeshEmpty())return;
  Mesh* mesh = sceneManager->get_selectedMesh();

  //---------------------------
  mesh->intensity.heatmap = false;
}

//Processing functions
void HeatMap::compute_applyHeatmap(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  vector<float>& dist = mesh->attribut.dist;
  vector<float>& cosIt = mesh->attribut.cosIt;
  vector<float>& It = mesh->attribut.It;

  if(dist.size() == 0) attribManager->compute_Distances(mesh);
  vector<float> dist_norm = Normalize(dist);
  vector<float>& dist_n = dist_norm;
  //---------------------------

  switch(HMmode){
    case 0:{//Is
      this->compute_heatmapColor(mesh, Is);
      break;
    }
    case 1:{//dist
      this->compute_heatmapColor(mesh, dist_n);
      break;
    }
    case 2:{//cosIt
      if(cosIt.size() == 0) attribManager->compute_cosIt(mesh);
      this->compute_heatmapColor(mesh, cosIt);
      break;
    }
    case 3:{//It
      if(It.size() == 0) attribManager->compute_cosIt(mesh);
      vector<float> It_norm = Normalize(It);
      vector<float>& It_n = It_norm;
      this->compute_heatmapColor(mesh, It_n);
      break;
    }
  }

  //---------------------------
}
void HeatMap::compute_reverseHeatmap(Mesh* mesh){
  int size = mesh->NbPoints;
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<float>& Is = mesh->intensity.OBJ;
  if(Is.size() == 0) return;
  //---------------------------

  for(int i=0; i<size; i++){
    RGB[i] = vec4(Is[i],Is[i],Is[i],1);
  }

  //---------------------------
}
void HeatMap::compute_heatmapColor(Mesh* mesh, vector<float>& v_in){
  vector<vec4>& RGB = mesh->color.OBJ;
  const int NUM_COLORS = 4;
  static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0} };
  //---------------------------

  int idx1;        // |-- Our desired color will be between these two indexes in "color".
  int idx2;        // |
  float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.
  float red, green, blue, alpha;

  vector<float> v_norm;
  if(normalized){
    v_norm = Normalize_01(v_in);
  }else{
    v_norm = v_in;
  }

  for(int i=0; i<RGB.size(); i++){
    if(v_in[i] != -1){
      float value = v_norm[i];
      value = value * (NUM_COLORS-1);        // Will multiply value by 3.
      idx1  = floor(value);                  // Our desired color will be after this index.
      idx2  = idx1+1;                        // ... and before this index (inclusive).
      fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).

      red   = (color[idx2][0] - color[idx1][0]) * fractBetween + color[idx1][0];
      green = (color[idx2][1] - color[idx1][1]) * fractBetween + color[idx1][1];
      blue  = (color[idx2][2] - color[idx1][2]) * fractBetween + color[idx1][2];

      RGB[i] = vec4(red, green, blue, 1.0f);
    }
    else{
      RGB[i] = vec4(1.0f, 1.0f, 1.0f, 1.0f);
    }
  }

  //---------------------------
}

//Plotting
void HeatMap::plot_colorPalette(Mesh* mesh){
  vector<vec4>& RGB = mesh->color.OBJ;
  vector<float> v_in;
  //---------------------------

  switch(HMmode){
    case 0:{ //Is
      v_in = mesh->intensity.OBJ;
      break;
    }
    case 1:{ //dist
      v_in = mesh->attribut.dist;
      break;
    }
    case 2:{ //cosIt
      v_in = mesh->attribut.cosIt;
      break;
    }
    case 3:{ //It
      v_in = mesh->attribut.It;
      break;
    }
  }

  //Pre-processing
  vector<float> v_Is;
  vector<vec4> v_RGB;
  for(int i=0; i<v_in.size(); i++){
    if(v_in[i] != -1){
      v_Is.push_back(v_in[i]);
      v_RGB.push_back(RGB[i]);
    }
  }

  vector<size_t> idx = sort_indexes(v_Is);
  sort(v_Is.begin(), v_Is.end());

  //Plotting
  bool flag = true;
  for(int i=0; i<2; i++){
    Gnuplot gp("tee '../graphs/gnuplot/PaletteColor.gp' | gnuplot -persist");

    if(flag){
      gp << "set terminal pngcairo\n";
      gp << "set output '../graphs/PaletteColor.png'\n";
      flag = false;
    }

    //Style
    gp << "g(y)=y\n";
    gp << "set format y '%-.2f'\n";
    gp << "unset key\n";

    gp << "set view map\n";
    gp << "set style data pm3d\n";
    gp << "set style function pm3d\n";
    gp << "set palette rgb 33,13,10\n";

    //gp << "set size ratio 2\n";
    gp << "set colorbox size 5,20\n";

    gp << "unset xtics\n";
    gp << "set ytics "<< Min(v_Is) <<","<< (Max(v_Is)-Min(v_Is))/10 <<","<< Max(v_Is) <<" scale 1.5 nomirror\n";
    gp << "unset ztics\n";
    gp << "set yrange ["<< Min(v_Is) <<":"<< Max(v_Is) <<"]\n";
    gp << "set mytics 2\n";
    //---------------------------------

    //Plot palette color
    gp << "set palette defined (";
    for(int i=0; i<v_Is.size(); i=i+v_Is.size()/1000){
      // #num R G B
      gp <<v_Is[i]<<" "<<v_RGB[idx[i]][0]<<" "<<v_RGB[idx[i]][1]<<" "<<v_RGB[idx[i]][2];
      if((i+v_Is.size()/1000) < v_Is.size()){
        gp << ", ";
      }else{
        gp << ")\n";
      }
    }

    gp << "splot g(y)\n";
  }

  //---------------------------
}
