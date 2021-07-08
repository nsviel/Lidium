#include "Radiometry.h"
#include "Target/Reference.h"
#include "Linearization.h"
#include "Target/Ref_Operation.h"

#include "Correction/RadarEquation.h"
#include "Correction/Surfacic_simplified.h"
#include "Correction/Surfacic_globalPiecewise.h"
#include "Correction/Separation_global.h"

#include "../Operation/Plotting.h"
#include "../Engine/Scene.h"
#include "../Operation/Attribut.h"
#include "../Operation/Functions/BundleByClass.h"

//plotting function
void Radiometry::plot_3Dmap(){
  refManager->compute_list(sceneManager->get_listMesh());
  list<Mesh*>* list_dist = refManager->get_listDist();
  list<Mesh*>* list_angle = refManager->get_listAngle();
  vector<vector<float>> data_X, data_Y, data_Z;
  //---------------------------------

  for(int i=0; i<list_dist->size(); i++){
    Mesh* mesh = *next(list_dist->begin(),i);

    //Bundle by statistical classes
    bundler->compute_bundleByClass(mesh, 5);
    vector<float> Ib = bundler->get_Ib();
    vector<float> Ib_dist = bundler->get_Ib_dist();
    vector<float> Ib_It = bundler->get_Ib_It();

    data_X.push_back(Ib_dist);
    data_Y.push_back(Ib_It);
    data_Z.push_back(Ib);
  }

  //---------------------------------
  plotManager->set_namePlot("3Dmap");
  plotManager->set_Xlabel("Distance R (m)");
  plotManager->set_Ylabel("Incidence angle {/Symbol a} (°)");
  plotManager->set_Zlabel("Intensity [0;1]");
  plotManager->set_Format_data1("with linespoints pt 7 ps 0.5 palette notitle");
  plotManager->plot_3Dmap(data_X, data_Y, data_Z);
}
void Radiometry::plot_2Dmap(){
  refManager->compute_list(sceneManager->get_listMesh());
  list<Mesh*>* list_dist = refManager->get_listDist();
  list<Mesh*>* list_angle = refManager->get_listAngle();
  this->compute_IRmeans(list_dist);
  MatrixXf heatmap(20, list_dist->size());
  vector<float> R_map, cosIt_map;
  //--------------------------

  for(int i=0; i<list_dist->size(); i++){
    Mesh* mesh = *next(list_dist->begin(),i);
    vector<float>& Is = mesh->intensity.OBJ;
    vector<float>& cosIt = mesh->attribut.cosIt;
    vector<float>& dist = mesh->attribut.dist;

    //Store data
    int cpt = 0;
    for(int j=0; j<Is.size(); j=j+Is.size()/20){
      heatmap(cpt,i) = Is[j];
      cosIt_map.push_back(cosIt[j]);
      R_map.push_back(dist[j]);
      cpt++;
    }
  }

  //--------------------------
  plotManager->set_namePlot("2Dmap");
  plotManager->set_Xlabel("R[m]");
  plotManager->set_Ylabel("cos({/Symbol a})");
  plotManager->set_Zlabel("I_{raw}");
  plotManager->plot_2Dmap(heatmap, R_map, cosIt_map);
}
void Radiometry::plot_SpectralonAllMeans(){
  vector<vector<float>> Is_99, Is_50, Is_25, Is_10;
  console.AddLog("Plotting Spectralon all means...");
  //---------------------------

  refopeManager->compute_Spectralon_IbyR();
  Is_99.push_back(refopeManager->get_Spec_IbyR_I01_99());
  Is_50.push_back(refopeManager->get_Spec_IbyR_I01_50());
  Is_25.push_back(refopeManager->get_Spec_IbyR_I01_25());
  Is_10.push_back(refopeManager->get_Spec_IbyR_I01_10());

  if(refopeManager->compute_Spectralon_IbycIt(10)){
    Is_99.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_50.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_25.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_10.push_back(refopeManager->get_Spec_IbycIt_I01_10());
  }
  if(refopeManager->compute_Spectralon_IbycIt(20)){
    Is_99.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_50.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_25.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_10.push_back(refopeManager->get_Spec_IbycIt_I01_10());
  }
  if(refopeManager->compute_Spectralon_IbycIt(30)){
    Is_99.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_50.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_25.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_10.push_back(refopeManager->get_Spec_IbycIt_I01_10());
  }
  if(refopeManager->compute_Spectralon_IbycIt(40)){
    Is_99.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_50.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_25.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_10.push_back(refopeManager->get_Spec_IbycIt_I01_10());
  }
  console.AddLog("Data acquiered...ok");

  //Data concatenation
  vector<float> rho, Icor;
  for(int i=0; i<Is_99.size(); i++){
    Icor.insert( Icor.end(), Is_99[i].begin(), Is_99[i].end() );
  }
  int size = Icor.size();
  for(int i=0; i<Is_50.size(); i++){
    Icor.insert( Icor.end(), Is_50[i].begin(), Is_50[i].end() );
  }
  for(int i=0; i<Is_25.size(); i++){
    Icor.insert( Icor.end(), Is_25[i].begin(), Is_25[i].end() );
  }
  for(int i=0; i<Is_10.size(); i++){
    Icor.insert( Icor.end(), Is_10[i].begin(), Is_10[i].end() );
  }
  for(int i=0; i<size; i++){
    rho.push_back(99);
  }
  for(int i=0; i<size; i++){
    rho.push_back(57);
  }
  for(int i=0; i<size; i++){
    rho.push_back(28);
  }
  for(int i=0; i<size; i++){
    rho.push_back(11);
  }
  console.AddLog("Concatenation...ok");

  //plotting
  plotManager->set_keyOn(false);
  plotManager->set_keyOutside(false);
  plotManager->set_labelOn(true);
  plotManager->set_namePlot("IbyRho_Spectralon");
  plotManager->set_Xlabel("Calibrated reflectance");
  plotManager->set_Ylabel("I_{cor_{mean}}");
  plotManager->set_xRange("[0:100]");
  plotManager->set_yRange("[0:1]");
  plotManager->set_ticSize(8);
  plotManager->set_Format_data1("pt 7 ps 0.1 lc rgb '" color_a1 "' notitle");
  plotManager->plot_Curve(rho, Icor);
  console.AddLog("Plotting...ok");

  //Write data on file
  ofstream file;
  file.open ("../data/data/Spectralon_data.txt");
  file << "Rho_th "<<"R "<<"alpha "<<"Icor "<<"std "<<"Nbpoint "<<"\n";
  file << "--------------------------------"<<"\n";

  //------------IbyR------------
  vector<float> Is, R, std, It, nb;

  //99%
  refopeManager->compute_Spectralon_IbyR();
  Is = refopeManager->get_Spec_IbyR_I01_99();
  R = refopeManager->get_Spec_IbyR_R_99();
  std = refopeManager->get_Spec_IbyR_std_99();
  It = refopeManager->get_Spec_IbyR_It_99();
  nb = refopeManager->get_Spec_IbyR_nb_99();
  file <<std::fixed;
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "99 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(10);
  Is = refopeManager->get_Spec_IbycIt_I01_99();
  R = refopeManager->get_Spec_IbycIt_R_99();
  std = refopeManager->get_Spec_IbycIt_std_99();
  It = refopeManager->get_Spec_IbycIt_It_99();
  nb = refopeManager->get_Spec_IbycIt_nb_99();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "99 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(20);
  Is = refopeManager->get_Spec_IbycIt_I01_99();
  R = refopeManager->get_Spec_IbycIt_R_99();
  std = refopeManager->get_Spec_IbycIt_std_99();
  It = refopeManager->get_Spec_IbycIt_It_99();
  nb = refopeManager->get_Spec_IbycIt_nb_99();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "99 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(30);
  Is = refopeManager->get_Spec_IbycIt_I01_99();
  R = refopeManager->get_Spec_IbycIt_R_99();
  std = refopeManager->get_Spec_IbycIt_std_99();
  It = refopeManager->get_Spec_IbycIt_It_99();
  nb = refopeManager->get_Spec_IbycIt_nb_99();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "99 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(40);
  Is = refopeManager->get_Spec_IbycIt_I01_99();
  R = refopeManager->get_Spec_IbycIt_R_99();
  std = refopeManager->get_Spec_IbycIt_std_99();
  It = refopeManager->get_Spec_IbycIt_It_99();
  nb = refopeManager->get_Spec_IbycIt_nb_99();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "99 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  //57%
  refopeManager->compute_Spectralon_IbyR();
  Is = refopeManager->get_Spec_IbyR_I01_50();
  R = refopeManager->get_Spec_IbyR_R_50();
  std = refopeManager->get_Spec_IbyR_std_50();
  It = refopeManager->get_Spec_IbyR_It_50();
  nb = refopeManager->get_Spec_IbyR_nb_50();
  file <<std::fixed;
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "57 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(10);
  Is = refopeManager->get_Spec_IbycIt_I01_50();
  R = refopeManager->get_Spec_IbycIt_R_50();
  std = refopeManager->get_Spec_IbycIt_std_50();
  It = refopeManager->get_Spec_IbycIt_It_50();
  nb = refopeManager->get_Spec_IbycIt_nb_50();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "57 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(20);
  Is = refopeManager->get_Spec_IbycIt_I01_50();
  R = refopeManager->get_Spec_IbycIt_R_50();
  std = refopeManager->get_Spec_IbycIt_std_50();
  It = refopeManager->get_Spec_IbycIt_It_50();
  nb = refopeManager->get_Spec_IbycIt_nb_50();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "57 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(30);
  Is = refopeManager->get_Spec_IbycIt_I01_50();
  R = refopeManager->get_Spec_IbycIt_R_50();
  std = refopeManager->get_Spec_IbycIt_std_50();
  It = refopeManager->get_Spec_IbycIt_It_50();
  nb = refopeManager->get_Spec_IbycIt_nb_50();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "57 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(40);
  Is = refopeManager->get_Spec_IbycIt_I01_50();
  R = refopeManager->get_Spec_IbycIt_R_50();
  std = refopeManager->get_Spec_IbycIt_std_50();
  It = refopeManager->get_Spec_IbycIt_It_50();
  nb = refopeManager->get_Spec_IbycIt_nb_50();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "57 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  //28%
  refopeManager->compute_Spectralon_IbyR();
  Is = refopeManager->get_Spec_IbyR_I01_25();
  R = refopeManager->get_Spec_IbyR_R_25();
  std = refopeManager->get_Spec_IbyR_std_25();
  It = refopeManager->get_Spec_IbyR_It_25();
  nb = refopeManager->get_Spec_IbyR_nb_25();
  file <<std::fixed;
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "28 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(10);
  Is = refopeManager->get_Spec_IbycIt_I01_25();
  R = refopeManager->get_Spec_IbycIt_R_25();
  std = refopeManager->get_Spec_IbycIt_std_25();
  It = refopeManager->get_Spec_IbycIt_It_25();
  nb = refopeManager->get_Spec_IbycIt_nb_25();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "28 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(20);
  Is = refopeManager->get_Spec_IbycIt_I01_25();
  R = refopeManager->get_Spec_IbycIt_R_25();
  std = refopeManager->get_Spec_IbycIt_std_25();
  It = refopeManager->get_Spec_IbycIt_It_25();
  nb = refopeManager->get_Spec_IbycIt_nb_25();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "28 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(30);
  Is = refopeManager->get_Spec_IbycIt_I01_25();
  R = refopeManager->get_Spec_IbycIt_R_25();
  std = refopeManager->get_Spec_IbycIt_std_25();
  It = refopeManager->get_Spec_IbycIt_It_25();
  nb = refopeManager->get_Spec_IbycIt_nb_25();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "28 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(40);
  Is = refopeManager->get_Spec_IbycIt_I01_25();
  R = refopeManager->get_Spec_IbycIt_R_25();
  std = refopeManager->get_Spec_IbycIt_std_25();
  It = refopeManager->get_Spec_IbycIt_It_25();
  nb = refopeManager->get_Spec_IbycIt_nb_25();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "28 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  //11%
  refopeManager->compute_Spectralon_IbyR();
  Is = refopeManager->get_Spec_IbyR_I01_10();
  R = refopeManager->get_Spec_IbyR_R_10();
  std = refopeManager->get_Spec_IbyR_std_10();
  It = refopeManager->get_Spec_IbyR_It_10();
  nb = refopeManager->get_Spec_IbyR_nb_10();
  file <<std::fixed;
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "11 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(10);
  Is = refopeManager->get_Spec_IbycIt_I01_10();
  R = refopeManager->get_Spec_IbycIt_R_10();
  std = refopeManager->get_Spec_IbycIt_std_10();
  It = refopeManager->get_Spec_IbycIt_It_10();
  nb = refopeManager->get_Spec_IbycIt_nb_10();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "11 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(20);
  Is = refopeManager->get_Spec_IbycIt_I01_10();
  R = refopeManager->get_Spec_IbycIt_R_10();
  std = refopeManager->get_Spec_IbycIt_std_10();
  It = refopeManager->get_Spec_IbycIt_It_10();
  nb = refopeManager->get_Spec_IbycIt_nb_10();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "11 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(30);
  Is = refopeManager->get_Spec_IbycIt_I01_10();
  R = refopeManager->get_Spec_IbycIt_R_10();
  std = refopeManager->get_Spec_IbycIt_std_10();
  It = refopeManager->get_Spec_IbycIt_It_10();
  nb = refopeManager->get_Spec_IbycIt_nb_10();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "11 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  refopeManager->compute_Spectralon_IbycIt(40);
  Is = refopeManager->get_Spec_IbycIt_I01_10();
  R = refopeManager->get_Spec_IbycIt_R_10();
  std = refopeManager->get_Spec_IbycIt_std_10();
  It = refopeManager->get_Spec_IbycIt_It_10();
  nb = refopeManager->get_Spec_IbycIt_nb_10();
  for(int i=0; i<Is.size(); i++){
    file <<setprecision(5)<< "11 "<<R[i]<<" "<<It[i]<<" "<<Is[i]<<" "<<std[i]<<" "<<nb[i]<<"\n";
  }

  //---------------------------
  file.close();
}

//IbyR
void Radiometry::plot_IbyR(){
  //---------------------------

  //If spectralon targets
  if(refManager->isref_Spectralon()){
    this->plot_IbyR_Spectralon();
  }
  //If sphere targets
  else if(refManager->isref_Sphere()){
    this->plot_IbyR_Sphere();
  }
  //If spectralon & sphere targets
  else if(refManager->isref_Spectralon() && refManager->isref_Sphere()){
    this->plot_IbyR_both();
  }
  //else plot all loaded clouds
  else{
    this->plot_IbyR_all();
  }

  this->plot_IbyR_all();

  //---------------------------
}
void Radiometry::plot_IbyR_all(){
  list<Mesh*>* list = sceneManager->get_listMesh();
  vector<float> Is, Is_std, Is_R;
  //---------------------------

  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);
    attribManager->compute_meshAttributs(mesh);

    bundler->compute_bundleByClass_maxAngle(mesh, 5, 15);
    vector<float> Ib_01 = bundler->get_Ib();
    vector<float> Ib_R = bundler->get_Ib_dist();

    Is.push_back(fct_Mean(Ib_01));
    Is_R.push_back(fct_Mean(Ib_R));
    Is_std.push_back(fct_std(Ib_01));
  }

  //Plotting
  plotManager->set_keyOn(false);
  plotManager->set_keyOutside(false);
  plotManager->set_labelOn(false);
  plotManager->set_namePlot("Sphere_IbyR");
  plotManager->set_Xlabel("R_{mean}");
  plotManager->set_Ylabel("I_{mean}");
  plotManager->set_xRange("[0:*]");
  plotManager->set_yRange("[0.7:0.95]");
  plotManager->set_Format_data1("pt 7 ps 0.5 lc rgb '" color_a1 "' notitle");
  plotManager->plot_CurveError(Is_R, Is, Is_std);

  //---------------------------
}
void Radiometry::plot_IbyR_Sphere(){
  //refManager->compute_list(sceneManager->get_listMesh());
  refopeManager->compute_Sphere_IbyR();
  vector<float> IbyR_R = refopeManager->get_Sphere_IbyR_R();
  vector<float> IbyR_I = refopeManager->get_Sphere_IbyR_I01();
  vector<float> IbyR_std = refopeManager->get_Sphere_IbyR_I01_std();
  //---------------------------

  //Write data on file
  ofstream file;
  file.open ("../data/data/IbyR_Sphere.txt");
  file << "R "<<"I "<<"std"<<"\n";
  file << "--------------------------------"<<"\n";
  file <<std::fixed;
  for(int i=0; i<IbyR_R.size(); i++){
    file <<setprecision(5)<< IbyR_R[i]<<" "<<IbyR_I[i]<<" "<<IbyR_std[i]<<"\n";
  }
  file.close();

  //Plotting
  plotManager->set_keyOn(false);
  plotManager->set_keyOutside(false);
  plotManager->set_labelOn(false);
  plotManager->set_namePlot("Sphere_IbyR");
  plotManager->set_Xlabel("R_{mean}");
  plotManager->set_Ylabel("I_{mean}");
  plotManager->set_xRange("[0:*]");
  plotManager->set_yRange("[0.7:0.95]");
  plotManager->set_Format_data1("pt 7 ps 0.5 lc rgb '" color_a1 "' notitle");
  plotManager->plot_CurveError(IbyR_R, IbyR_I, IbyR_std);

  //---------------------------
  console.AddLog("IbyR Intensity : I=%.2f std=%.2f CV=%.2f", fct_Mean(IbyR_I), fct_std(IbyR_I), fct_CV(IbyR_I));
}
void Radiometry::plot_IbyR_Spectralon(){
  vector<vector<float>> Is_multi, R_multi, std_multi;
  refopeManager->compute_Spectralon_IbyR();
  //---------------------------

  R_multi.push_back(refopeManager->get_Spec_IbyR_R_99());
  R_multi.push_back(refopeManager->get_Spec_IbyR_R_50());
  R_multi.push_back(refopeManager->get_Spec_IbyR_R_25());
  R_multi.push_back(refopeManager->get_Spec_IbyR_R_10());

  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_99());
  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_50());
  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_25());
  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_10());

  std_multi.push_back(refopeManager->get_Spec_IbyR_std_99());
  std_multi.push_back(refopeManager->get_Spec_IbyR_std_50());
  std_multi.push_back(refopeManager->get_Spec_IbyR_std_25());
  std_multi.push_back(refopeManager->get_Spec_IbyR_std_10());
  if(R_multi.size() == Is_multi.size() && R_multi.size() == std_multi.size() && R_multi.size() != 0){
    say("---> Data ok");
  }

  //Write data on file
  ofstream file;
  file.open ("../data/data/IbyR_Spectralon.txt");
  file << "#R99 R50 R25 R10 It99 It50 It25 It10 "<<"99% "<<"57% "<<"28% "<<"11% "<<"std(99%) "<<"std(57%) "<<"std(28%) "<<"std(11%)"<<"\n";
  file << "#--------------------------------"<<"\n";
  vector<float> R_99 = refopeManager->get_Spec_IbyR_R_99();
  vector<float> R_50 = refopeManager->get_Spec_IbyR_R_50();
  vector<float> R_25 = refopeManager->get_Spec_IbyR_R_25();
  vector<float> R_10 = refopeManager->get_Spec_IbyR_R_10();

  vector<float> I_99 = refopeManager->get_Spec_IbyR_I01_99();
  vector<float> I_50 = refopeManager->get_Spec_IbyR_I01_50();
  vector<float> I_25 = refopeManager->get_Spec_IbyR_I01_25();
  vector<float> I_10 = refopeManager->get_Spec_IbyR_I01_10();

  vector<float> std_99 = refopeManager->get_Spec_IbyR_std_99();
  vector<float> std_50 = refopeManager->get_Spec_IbyR_std_50();
  vector<float> std_25 = refopeManager->get_Spec_IbyR_std_25();
  vector<float> std_10 = refopeManager->get_Spec_IbyR_std_10();
  file <<std::fixed;
  for(int i=0; i<I_99.size(); i++){
    file <<setprecision(5) << R_99[i] << " " << R_50[i] << " "<< R_25[i] << " " << R_10[i] << " ";
    file <<setprecision(5) << I_99[i] << " " << I_50[i] << " "<< I_25[i] << " " << I_10[i] << " ";
    file <<setprecision(5) << std_99[i] << " " << std_50[i] << " " << std_25[i] << " " << std_10[i] << "\n";
  }
  file.close();

  vector<string> dataFormA;
  dataFormA.push_back("ls 1 pt 7 ps 0.5 lc rgb '" color_a1 "' title '99%'");
  dataFormA.push_back("ls 1 pt 5 ps 0.5 lc rgb '" color_a2 "' title '57%'");
  dataFormA.push_back("ls 1 pt 9 ps 0.5 lc rgb '" color_a3 "' title '28%'");
  dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb '" color_a4 "' title '11%'");

  //Plotting
  plotManager->set_keyOn(true);
  plotManager->set_keyOutside(false);
  plotManager->set_labelOn(false);
  plotManager->set_namePlot("Spectralon_IbyR");
  plotManager->set_Xlabel("Distance R (m)");
  plotManager->set_Ylabel("Intensity I [0;1]");
  plotManager->set_xRange("[0:*]");
  plotManager->set_yRange("[0.4:1.1]");
  plotManager->set_dataFormA(dataFormA);
  plotManager->plot_Curve_Multi_Err(R_multi, Is_multi, std_multi);

  //---------------------------
  console.AddLog("IbyR Spectralon 99%%: I=%.2f std=%.2f CV=%.2f", fct_Mean(Is_multi[0]), fct_std(Is_multi[0]), fct_CV(Is_multi[0]));
  console.AddLog("IbyR Spectralon 50%%: I=%.2f std=%.2f CV=%.2f", fct_Mean(Is_multi[1]), fct_std(Is_multi[1]), fct_CV(Is_multi[1]));
  console.AddLog("IbyR Spectralon 25%%: I=%.2f std=%.2f CV=%.2f", fct_Mean(Is_multi[2]), fct_std(Is_multi[2]), fct_CV(Is_multi[2]));
  console.AddLog("IbyR Spectralon 10%%: I=%.2f std=%.2f CV=%.2f", fct_Mean(Is_multi[3]), fct_std(Is_multi[3]), fct_CV(Is_multi[3]));
}
void Radiometry::plot_IbyR_both(){
  vector<vector<float>> Is_multi, R_multi, std_multi;
  refopeManager->compute_Spectralon_IbyR();
  //---------------------------

  R_multi.push_back(refopeManager->get_Spec_IbyR_R_99());
  R_multi.push_back(refopeManager->get_Spec_IbyR_R_50());
  R_multi.push_back(refopeManager->get_Spec_IbyR_R_25());
  R_multi.push_back(refopeManager->get_Spec_IbyR_R_10());

  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_99());
  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_50());
  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_25());
  Is_multi.push_back(refopeManager->get_Spec_IbyR_I01_10());

  std_multi.push_back(refopeManager->get_Spec_IbyR_std_99());
  std_multi.push_back(refopeManager->get_Spec_IbyR_std_50());
  std_multi.push_back(refopeManager->get_Spec_IbyR_std_25());
  std_multi.push_back(refopeManager->get_Spec_IbyR_std_10());

  refManager->compute_list(sceneManager->get_listMesh());
  refopeManager->compute_Sphere_IbyR();
  R_multi.push_back(refopeManager->get_Sphere_IbyR_R());
  Is_multi.push_back(refopeManager->get_Sphere_IbyR_I01());
  std_multi.push_back(refopeManager->get_Sphere_IbyR_I01_std());

  vector<string> dataFormA;
  dataFormA.push_back("ls 1 pt 7 ps 0.5 lc rgb '" color_a1 "' title '99%'");
  dataFormA.push_back("ls 1 pt 5 ps 0.5 lc rgb '" color_a2 "' title '57%'");
  dataFormA.push_back("ls 1 pt 9 ps 0.5 lc rgb '" color_a3 "' title '28%'");
  dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb '" color_a4 "' title '11%'");
  dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb 'red' title 'Sphere'");

  //Plotting
  plotManager->set_keyOn(true);
  plotManager->set_keyOutside(false);
  plotManager->set_labelOn(false);
  plotManager->set_namePlot("IbyR_Sphere+Spectralon");
  plotManager->set_Xlabel("Distance R (m)");
  plotManager->set_Ylabel("Intensity I [0;1]");
  plotManager->set_xRange("[0:*]");
  plotManager->set_yRange("[0.4:1.1]");

  //---------------------------
  plotManager->set_dataFormA(dataFormA);
  plotManager->plot_Curve_Multi_Err(R_multi, Is_multi, std_multi);
}

//IbyIt
void Radiometry::plot_IbyCosIt(bool normalised){
  refManager->compute_list(sceneManager->get_listMesh());
  //---------------------------

  if(refManager->isref_Spectralon()){
    this->plot_IbyIt_Spectralon(normalised);
  }
  if(refManager->isref_Sphere()){
    this->plot_IbyCosIt_Sphere(normalised);
    this->plot_IbyIt_Sphere(normalised);
  }

  //---------------------------
}
void Radiometry::plot_IbyIt_Spectralon(bool normalised){
  list<Mesh*>* list_xp_10m_xd = refManager->get_list_xp_10m_xd();
  list<Mesh*>* list_xp_20m_xd = refManager->get_list_xp_20m_xd();
  list<Mesh*>* list_xp_30m_xd = refManager->get_list_xp_30m_xd();
  list<Mesh*>* list_xp_40m_xd = refManager->get_list_xp_40m_xd();
  //---------------------------

  //Compute separate lists
  vector<vector<float>> Is_multi, Is_multi_n, It_multi;
  vector<string> title;
  vector<string> color;
  if(list_xp_10m_xd->size() > 0){
    this->compute_IRmeans(list_xp_10m_xd);
    Is_multi.push_back(Is_mean);
    Is_multi_n.push_back(Normalize(Is_mean));
    It_multi.push_back(It_mean);

    title.push_back("title 'I_{99}=f({/Symbol a}) - 10m'");
    color.push_back(color_a1);
  }
  if(list_xp_20m_xd->size() > 0){
    this->compute_IRmeans(list_xp_20m_xd);
    Is_multi.push_back(Is_mean);
    Is_multi_n.push_back(Normalize(Is_mean));
    It_multi.push_back(It_mean);

    title.push_back("title 'I_{99}=f({/Symbol a}) - 20m'");
    color.push_back(color_a2);
  }
  if(list_xp_30m_xd->size() > 0){
    this->compute_IRmeans(list_xp_30m_xd);
    Is_multi.push_back(Is_mean);
    Is_multi_n.push_back(Normalize(Is_mean));
    It_multi.push_back(It_mean);

    title.push_back("title 'I_{99}=f({/Symbol a}) - 30m'");
    color.push_back(color_a3);
  }
  if(list_xp_40m_xd->size() > 0){
    this->compute_IRmeans(list_xp_40m_xd);
    Is_multi.push_back(Is_mean);
    Is_multi_n.push_back(Normalize(Is_mean));
    It_multi.push_back(It_mean);

    title.push_back("title 'I_{99}=f({/Symbol a}) - 30m'");
    color.push_back(color_a4);
  }

  //Plotting
  plotManager->set_namePlot("IbyIt_Spectralon");
  plotManager->set_Xlabel("{/Symbol a}");
  plotManager->set_Ylabel("I_{mean}");
  plotManager->set_xRange("[0:*]");
  plotManager->set_yRange("[0:1]");

  //---------------------------
  if(normalised){
    //Cosine law
    vector<float> cosLaw;
    for(int i=0; i<It_mean.size(); i++){
      float alpha = It_mean[i] * M_PI / 180;
      cosLaw.push_back(cos(alpha));
    }
    It_multi.insert(It_multi.begin(), It_mean);
    Is_multi_n.insert(Is_multi_n.begin(), cosLaw);
    color.insert(color.begin(), "red");
    title.insert(title.begin(), "title 'Cosine law'");

    //Plotting
    plotManager->set_dataTitle(title);
    plotManager->set_dataColor(color);
    plotManager->plot_Curve_Multi(It_multi, Is_multi_n);
  }else{
    if(It_multi.size() == 1){
      plotManager->plot_Curve(It_mean, Is_mean);
    }else{
      plotManager->plot_Curve_Multi(It_multi, Is_multi);
    }
  }
}
void Radiometry::plot_IbyCosIt_Sphere(bool normalised){
  Mesh* mesh = sceneManager->get_selectedMesh();
  //---------------------------

  //Bundle by classes
  bundler->compute_bundleByClass(mesh, 5);
  vector<float> Ib = bundler->get_Ib();
  vector<float> Ib_cosIt = bundler->get_Ib_cosIt();
  vector<float> Ib_It = bundler->get_Ib_It();
  vector<float> Ib_std = bundler->get_Ib_std();

  //Cosine law
  vector<float> cosLaw;
  for(int i=0; i<Ib_It.size(); i++){
    float alpha = Ib_It[i] * M_PI / 180;
    cosLaw.push_back(cos(alpha));
  }

  //Plotting
  plotManager->set_namePlot("IbyCosIt_Sphere");
  plotManager->set_Xlabel("cos({/Symbol a})");
  plotManager->set_Ylabel("I_{raw}");
  plotManager->set_Format_data1("pt 7 ps 0.5 lc rgb 'black' notitle");
  plotManager->set_Format_data2("with lines smooth unique lc rgb '#cc0000' title 'Lambert cosine law'");

  //---------------------------
  if(normalised == false){
    plotManager->plot_CurveError(Ib_cosIt, Ib, Ib_std);
  }else{
    vector<float> nIb = Normalize(Ib);
    plotManager->plot_2CurvesError(Ib_cosIt, nIb, Ib_std, Ib_cosIt, cosLaw);
  }
}
void Radiometry::plot_IbyIt_Sphere(bool normalised){
  Mesh* mesh = sceneManager->get_selectedMesh();
  //---------------------------

  //Bundle by classes
  bundler->compute_bundleByClass(mesh, 5);
  vector<float> Ib = bundler->get_Ib();
  vector<float> Ib_cosIt = bundler->get_Ib_cosIt();
  vector<float> Ib_It = bundler->get_Ib_It();
  vector<float> Ib_std = bundler->get_Ib_std();

  //Write data on file
  ofstream file;
  file.open ("../data/data/IbyIt_Sphere.txt");
  file << "It "<<"I "<<"std"<<"\n";
  file << "--------------------------------"<<"\n";
  file <<std::fixed;
  for(int i=0; i<Ib_It.size(); i++){
    file <<setprecision(5)<< Ib_It[i]<<" "<<Ib[i]<<" "<<Ib_std[i]<<"\n";
  }
  file.close();

  //Plotting
  plotManager->set_namePlot("IbyIt_Sphere");
  plotManager->set_Xlabel("{/Symbol a}");
  plotManager->set_Ylabel("I_{raw}");
  plotManager->set_xRange("[0:*]");
  //plotManager->set_yRange("[0.8:1]");
  plotManager->set_Format_data1("pt 7 ps 0.5 lc rgb '" color_a1 "' notitle");
  plotManager->set_Format_data2("with lines smooth unique lc rgb '#cc0000' title 'Lambert cosine law'");

  //---------------------------
  if(normalised == false){
    plotManager->plot_CurveError(Ib_It, Ib, Ib_std);
  }else{
    vector<float> nIb = Normalize(Ib);
    plotManager->plot_CurveError(Ib_It, nIb, Ib_std);
  }
}

//IbyCosIt
void Radiometry::plot_IbyCosIt_all(){
  //---------------------------

this->plot_IbyIt_allPercentage_Spectralon_10m();

  if(refManager->isref_Spectralon()){
    this->plot_IbyIt_allPercentage_Spectralon_05m();
    this->plot_IbyIt_allPercentage_Spectralon_10m();
    this->plot_IbyIt_allPercentage_Spectralon_20m();
    this->plot_IbyIt_allPercentage_Spectralon_30m();
    this->plot_IbyIt_allPercentage_Spectralon_40m();
    //this->plot_IbyIt_allAngle_Spectralon(false);
  }
  else if(refManager->isref_Sphere()){
    //this->plot_IbyCosIt_all_Sphere();
    //this->plot_IbyIt_all_Sphere();
  }
  else{
    refManager->compute_list(sceneManager->get_listMesh());
  }

  //---------------------------
}
void Radiometry::plot_IbyCosIt_all_Sphere(){
  list<Mesh*>* list = refManager->get_listSphere();
  vector<vector<float>> data_X, data_Y;
  //---------------------------

  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);

    //Bundle by statistical classes
    bundler->compute_bundleByClass(mesh, 5);
    vector<float> Ib_cosIt = bundler->get_Ib_cosIt();
    vector<float> Ib = bundler->get_Ib();

    data_X.push_back(Ib_cosIt);
    data_Y.push_back(Ib);
  }

  //Compute classes
  bundler->compute_vectorByClass(data_X, data_Y);
  vector<float> data_Xb = bundler->get_vecb_dX();
  vector<float> data_Yb = bundler->get_vecb_dY();
  vector<float> err = bundler->get_vecb_std();

  //Plotting
  plotManager->set_namePlot("IsbyCosIt_all_Sphere");
  plotManager->set_Xlabel("cos({/Symbol a})");
  plotManager->set_Ylabel("I_{raw}");
  plotManager->set_Format_data1("pt 7 ps 0.5 lw 0.5 lc rgb 'black' title 'I=f({/Symbol a})'");
  plotManager->plot_CurveError(data_Xb, data_Yb, err);

  //---------------------------
}
void Radiometry::plot_IbyIt_all_Sphere(){
  refManager->compute_list(sceneManager->get_listMesh());
  //---------------------------

  refopeManager->compute_Sphere_IbycIt();
  vector<vector<float>> data_X = refopeManager->get_Sphere_IbycIt_It();
  vector<vector<float>> data_Y = refopeManager->get_Sphere_IbycIt_I01();

  //Compute classes
  bundler->compute_vectorByClass(data_X, data_Y);
  vector<float> data_Xb = bundler->get_vecb_dX();
  vector<float> data_Yb = bundler->get_vecb_dY();
  vector<float> err = bundler->get_vecb_std();

  //Plotting
  plotManager->set_namePlot("IsbyIt_all_Sphere");
  plotManager->set_Xlabel("{/Symbol a}");
  plotManager->set_Ylabel("I_{raw}");
  plotManager->set_Format_data1("pt 7 ps 0.5 lw 0.5 lc rgb '" color_a1 "' title 'I=f({/Symbol a})'");
  plotManager->plot_CurveError(data_Xb, data_Yb, err);

  //---------------------------
}
void Radiometry::plot_IbyIt_allAngle_Spectralon(bool normalised){
  list<Mesh*>* list_xp_10m_xd = refManager->get_list_xp_10m_xd();
  list<Mesh*>* list_xp_20m_xd = refManager->get_list_xp_20m_xd();
  list<Mesh*>* list_xp_30m_xd = refManager->get_list_xp_30m_xd();
  list<Mesh*>* list_xp_40m_xd = refManager->get_list_xp_40m_xd();
  vector<vector<float>> Is_multi, Is_multi_n, It_multi;
  //---------------------------

  if(list_xp_10m_xd->size() > 0){
    this->compute_IsItconcat(list_xp_10m_xd);
    Is_multi.push_back(Is_conc);
    Is_multi_n.push_back(Normalize(Is_conc));
    It_multi.push_back(It_conc);
  }
  if(list_xp_20m_xd->size() > 0){
    this->compute_IsItconcat(list_xp_20m_xd);
    Is_multi.push_back(Is_conc);
    Is_multi_n.push_back(Normalize(Is_conc));
    It_multi.push_back(It_conc);
  }
  if(list_xp_30m_xd->size() > 0){
    this->compute_IsItconcat(list_xp_30m_xd);
    Is_multi.push_back(Is_conc);
    Is_multi_n.push_back(Normalize(Is_conc));
    It_multi.push_back(It_conc);
  }
  if(list_xp_40m_xd->size() > 0){
    this->compute_IsItconcat(list_xp_40m_xd);
    Is_multi.push_back(Is_conc);
    Is_multi_n.push_back(Normalize(Is_conc));
    It_multi.push_back(It_conc);
  }

  //Plotting options
  plotManager->set_namePlot("IbyIt_allAngle_Spectralon");
  plotManager->set_Xlabel("{/Symbol a}_{mean}");
  plotManager->set_Ylabel("I_{mean}");

  //---------------------------
  if(normalised){
    vector<float> cosLaw;
    for(int i=0; i<It_mean.size(); i++){
      float alpha = It_mean[i] * M_PI / 180;
      cosLaw.push_back(cos(alpha));
    }
    It_multi.insert(It_multi.begin(), It_mean);
    Is_multi_n.insert(Is_multi_n.begin(), cosLaw);

    vector<string> title;
    title.push_back("Cosine law");
    title.push_back("I_{99}=f({/Symbol a})");
    vector<string> color;
    color.push_back("red");
    color.push_back("black");
    plotManager->set_dataColor(color);
    plotManager->plot_Curve_Multi(It_multi, Is_multi_n);
  }else{
    if(It_multi.size() == 1){
      plotManager->plot_Curve(It_mean, Is_mean);
    }else{
      plotManager->plot_Curve_Multi(It_multi, Is_multi);
    }
  }
}
void Radiometry::plot_IbyIt_allPercentage_Spectralon_05m(){
  vector<vector<float>> Is_multi, It_multi, std_multi;
  if(refopeManager->compute_Spectralon_IbycIt(05)){
    console.AddLog("IbyIt_Spectralon 05m...");
    //---------------------------

    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_99());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_50());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_25());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_10());

    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_10());

    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_99());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_50());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_25());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_10());
    if(It_multi.size() == Is_multi.size() && It_multi.size() == std_multi.size()){
      say("---> Data ok");
    }

    //Plotting
    vector<string> dataFormA;
    dataFormA.push_back("ls 1 pt 7 ps 0.5 lc rgb '" color_a1 "' title '5m 99%'");
    dataFormA.push_back("ls 1 pt 5 ps 0.5 lc rgb '" color_a2 "' title '5m 57%'");
    dataFormA.push_back("ls 1 pt 9 ps 0.5 lc rgb '" color_a3 "' title '5m 28%'");
    dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb '" color_a4 "' title '5m 11%'");

    plotManager->set_namePlot("IbyIt_allPercentage_Spectralon_05m");
    plotManager->set_Xlabel("Incidence angle {/Symbol a} (°)");
    plotManager->set_Ylabel("Intensity I [0;1]");
    plotManager->set_xRange("[0:*]");
    plotManager->set_yRange("[0.3:1]");
    plotManager->set_dataFormA(dataFormA);
    plotManager->plot_Curve_Multi_Err(It_multi, Is_multi, std_multi);

    //---------------------------
  }
}
void Radiometry::plot_IbyIt_allPercentage_Spectralon_10m(){
  vector<vector<float>> Is_multi, It_multi, std_multi;
  if(refopeManager->compute_Spectralon_IbycIt(10)){
    console.AddLog("IbyIt_Spectralon 10m...");
    //---------------------------

    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_99());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_50());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_25());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_10());

    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_10());

    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_99());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_50());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_25());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_10());
    if(Is_multi.size() != It_multi.size() && std_multi.size() != It_multi.size()){
      console.AddLog("IbyIt Spectralon 10m data problem");
    }

    //Save data in file
    ofstream file;
    file.open ("../data/data/IbyIt_Spectralon.txt");
    file << "It "<<"99% "<<"57% "<<"28% "<<"11% "<<"std(99%) "<<"std(57%) "<<"std(28%) "<<"std(11%)"<<"\n";
    file << "--------------------------------"<<"\n";
    vector<float> It_99 = refopeManager->get_Spec_IbycIt_It_99();
    vector<float> I_99 = refopeManager->get_Spec_IbycIt_I01_99();
    vector<float> I_50 = refopeManager->get_Spec_IbycIt_I01_50();
    vector<float> I_25 = refopeManager->get_Spec_IbycIt_I01_25();
    vector<float> I_10 = refopeManager->get_Spec_IbycIt_I01_10();

    vector<float> std_99 = refopeManager->get_Spec_IbycIt_std_99();
    vector<float> std_50 = refopeManager->get_Spec_IbycIt_std_50();
    vector<float> std_25 = refopeManager->get_Spec_IbycIt_std_25();
    vector<float> std_10 = refopeManager->get_Spec_IbycIt_std_10();

    file <<std::fixed;
    for(int i=0; i<It_99.size(); i++){
      file <<setprecision(5)<< It_99[i]<<" "<<I_99[i]<<" "<<I_50[i]<<" "<<I_25[i]<<" "<<I_10[i];
      file <<setprecision(5)<< " "<<std_99[i]<<" "<<std_50[i]<<" "<<std_25[i]<<" "<<std_10[i]<<"\n";
    }
    file.close();

    //Plotting
    plotManager->set_keyOn(true);
    plotManager->set_keyOutside(false);
    plotManager->set_labelOn(false);
    vector<string> dataFormA;
    dataFormA.push_back("ls 1 pt 7 ps 0.5 lc rgb '" color_a1 "' title '10m 99%'");
    dataFormA.push_back("ls 1 pt 5 ps 0.5 lc rgb '" color_a2 "' title '10m 57%'");
    dataFormA.push_back("ls 1 pt 9 ps 0.5 lc rgb '" color_a3 "' title '10m 28%'");
    dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb '" color_a4 "' title '10m 11%'");
    plotManager->set_dataFormA(dataFormA);
    plotManager->set_namePlot("Spectralon_IbyIt_10m");
    plotManager->set_Xlabel("Incidence angle {/Symbol a} (°)");
    plotManager->set_Ylabel("Intensity I [0;1]");
    plotManager->set_xRange("[0:*]");
    plotManager->set_yRange("[0.4:1.1]");
    plotManager->plot_Curve_Multi_Err(It_multi, Is_multi, std_multi);

    //---------------------------
    console.AddLog("IbyIt Spectralon 99%% 10m : I=%.3f std=%.3f CV=%.3f", fct_Mean(Is_multi[0]), fct_std(Is_multi[0]), fct_CV(Is_multi[0]));
    console.AddLog("IbyIt Spectralon 57%% 10m : I=%.3f std=%.3f CV=%.3f", fct_Mean(Is_multi[1]), fct_std(Is_multi[1]), fct_CV(Is_multi[1]));
    console.AddLog("IbyIt Spectralon 28%% 10m : I=%.3f std=%.3f CV=%.3f", fct_Mean(Is_multi[2]), fct_std(Is_multi[2]), fct_CV(Is_multi[2]));
    console.AddLog("IbyIt Spectralon 11%% 10m : I=%.3f std=%.3f CV=%.3f", fct_Mean(Is_multi[3]), fct_std(Is_multi[3]), fct_CV(Is_multi[3]));
  }
}
void Radiometry::plot_IbyIt_allPercentage_Spectralon_20m(){
  vector<vector<float>> Is_multi, It_multi, std_multi;
  if(refopeManager->compute_Spectralon_IbycIt(20)){
    console.AddLog("IbyIt_Spectralon 20m...");
    //---------------------------

    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_99());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_50());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_25());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_10());

    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_10());

    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_99());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_50());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_25());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_10());

    //Plotting
    plotManager->set_keyOn(true);
    plotManager->set_keyOutside(true);
    plotManager->set_labelOn(true);
    vector<string> dataFormA;
    dataFormA.push_back("ls 1 pt 7 ps 0.5 lc rgb '" color_a1 "' title '20m 99%'");
    dataFormA.push_back("ls 1 pt 5 ps 0.5 lc rgb '" color_a2 "' title '20m 57%'");
    dataFormA.push_back("ls 1 pt 9 ps 0.5 lc rgb '" color_a3 "' title '20m 28%'");
    dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb '" color_a4 "' title '20m 11%'");
    plotManager->set_dataFormA(dataFormA);
    plotManager->set_namePlot("IbyIt_Spectralon_20m");
    plotManager->set_Xlabel("Incidence angle {/Symbol a} (°)");
    plotManager->set_Ylabel("Intensity I [0;1]");
    plotManager->set_xRange("[0:*]");
    plotManager->set_yRange("[0.4:1.1]");
    plotManager->plot_Curve_Multi_Err(It_multi, Is_multi, std_multi);

    //---------------------------
  }
}
void Radiometry::plot_IbyIt_allPercentage_Spectralon_30m(){
  vector<vector<float>> Is_multi, It_multi, std_multi;
  if(refopeManager->compute_Spectralon_IbycIt(30)){
    console.AddLog("IbyIt_Spectralon 30m...");
    //---------------------------

    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_99());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_50());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_25());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_10());

    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_10());

    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_99());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_50());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_25());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_10());
    if(Is_multi.size() != It_multi.size() && std_multi.size() != It_multi.size()){
      console.AddLog("IbyIt Spectralon 30m data problem");
    }

    //Plotting
    plotManager->set_keyOn(true);
    plotManager->set_keyOutside(true);
    plotManager->set_labelOn(true);
    vector<string> dataFormA;
    dataFormA.push_back("ls 1 pt 7 ps 0.5 lc rgb '" color_a1 "' title '30m 99%'");
    dataFormA.push_back("ls 1 pt 5 ps 0.5 lc rgb '" color_a2 "' title '30m 57%'");
    dataFormA.push_back("ls 1 pt 9 ps 0.5 lc rgb '" color_a3 "' title '30m 28%'");
    dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb '" color_a4 "' title '30m 11%'");
    plotManager->set_dataFormA(dataFormA);
    plotManager->set_namePlot("IbyIt_Spectralon_30m");
    plotManager->set_Xlabel("Incidence angle {/Symbol a} (°)");
    plotManager->set_Ylabel("Intensity I [0;1]");
    plotManager->set_xRange("[0:*]");
    plotManager->set_yRange("[0.4:1.1]");
    plotManager->plot_Curve_Multi_Err(It_multi, Is_multi, std_multi);

    //---------------------------
  }
}
void Radiometry::plot_IbyIt_allPercentage_Spectralon_40m(){
  vector<vector<float>> Is_multi, It_multi, std_multi;
  if(refopeManager->compute_Spectralon_IbycIt(40)){
    console.AddLog("IbyIt_Spectralon 40m...");
    //---------------------------

    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_99());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_50());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_25());
    It_multi.push_back(refopeManager->get_Spec_IbycIt_It_10());

    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_99());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_50());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_25());
    Is_multi.push_back(refopeManager->get_Spec_IbycIt_I01_10());

    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_99());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_50());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_25());
    std_multi.push_back(refopeManager->get_Spec_IbycIt_std_10());
    if(Is_multi.size() != It_multi.size() && std_multi.size() != It_multi.size()){
      console.AddLog("IbyIt Spectralon 30m data problem");
    }

    //Plotting
    plotManager->set_keyOn(true);
    plotManager->set_keyOutside(true);
    plotManager->set_labelOn(true);
    vector<string> dataFormA;
    dataFormA.push_back("ls 1 pt 7 ps 0.5 lc rgb '" color_a1 "' title '30m 99%'");
    dataFormA.push_back("ls 1 pt 5 ps 0.5 lc rgb '" color_a2 "' title '40m 57%'");
    dataFormA.push_back("ls 1 pt 9 ps 0.5 lc rgb '" color_a3 "' title '40m 28%'");
    dataFormA.push_back("ls 1 pt 13 ps 0.5 lc rgb '" color_a4 "' title '40m 11%'");
    plotManager->set_dataFormA(dataFormA);
    plotManager->set_namePlot("IbyIt_Spectralon_40m");
    plotManager->set_Xlabel("Incidence angle {/Symbol a} (°)");
    plotManager->set_Ylabel("Intensity I [0;1]");
    plotManager->set_xRange("[0:*]");
    plotManager->set_yRange("[0.4:1.1]");
    plotManager->plot_Curve_Multi_Err(It_multi, Is_multi, std_multi);

    //---------------------------
  }
}

//mesh specific
void Radiometry::plot_IbyIt_mesh(Mesh* mesh){
  attribManager->compute_meshAttributs(mesh);
  //---------------------------

  //Bundle by classes
  bundler->compute_bundleByClass(mesh, 2);
  vector<float> Ib = bundler->get_Ib();
  vector<float> Ib_cosIt = bundler->get_Ib_cosIt();
  vector<float> Ib_It = bundler->get_Ib_It();
  vector<float> Ib_std = bundler->get_Ib_std();

  //Write data on file
  ofstream file;
  file.open ("../data/data/IbyIt_Sphere.txt");
  file << "It "<<"I "<<"std"<<"\n";
  file << "--------------------------------"<<"\n";
  file <<std::fixed;
  for(int i=0; i<Ib_It.size(); i++){
    file <<setprecision(5)<< Ib_It[i]<<" "<<Ib[i]<<" "<<Ib_std[i]<<"\n";
  }
  file.close();

  //Plotting
  plotManager->set_namePlot("IbyIt_Mesh");
  plotManager->set_Xlabel("Incidence angle {/Symbol a} (°)");
  plotManager->set_Ylabel("Intensity [0;1]");
  plotManager->set_xRange("[0:*]");
  plotManager->set_yRange("[0.7:0.95]");
  plotManager->set_Format_data1("pt 7 ps 0.5 lc rgb '" color_a1 "' notitle");
  plotManager->set_Format_data2("with lines smooth unique lc rgb '#cc0000' title 'Lambert cosine law'");
  plotManager->plot_CurveError(Ib_It, Ib, Ib_std);

  //---------------------------
  console.AddLog("IbyIt Intensity : I=%.3f std=%.3f CV=%.3f", fct_Mean(Ib), fct_std(Ib), fct_CV(Ib));
}
void Radiometry::plot_IbyR_mesh(Mesh* mesh){
  attribManager->compute_meshAttributs(mesh);
  //---------------------------

  //Bundle by classes
  bundler->compute_bundleByClass(mesh, 5);
  vector<float> Ib = bundler->get_Ib();
  vector<float> Ib_dist = bundler->get_Ib_dist();
  vector<float> Ib_std = bundler->get_Ib_std();

  //Plotting
  plotManager->set_namePlot("IbyR_Mesh");
  plotManager->set_Xlabel("R");
  plotManager->set_Ylabel("I");
  plotManager->set_Format_data1("pt 7 ps 0.5 lc rgb '" color_a1 "' notitle");
  plotManager->set_Format_data2("with lines smooth unique lc rgb '#cc0000' title 'Lambert cosine law'");
  plotManager->plot_CurveError(Ib_dist, Ib, Ib_std);

  //---------------------------
}
void Radiometry::plot_IbyR_data(Mesh* mesh){
  attribManager->compute_meshAttributs(mesh);
  vector<float>& R = mesh->attribut.dist;
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  //Plotting
  plotManager->set_namePlot("IbyR_data");
  plotManager->set_Xlabel("R");
  plotManager->set_Ylabel("I");
  plotManager->set_Format_data1("pt 7 ps 0.5 lc rgb '" color_a1 "' notitle");
  plotManager->set_Format_data2("with lines smooth unique lc rgb '#cc0000' title 'Lambert cosine law'");
  plotManager->plot_Curve(R, Is);

  //---------------------------
}

//Sphere specific
void Radiometry::plot_IbyItbyR(){
  list<Mesh*>* list = sceneManager->get_listMesh();
  vector<float> I_all, R_all, cIt_all;
  //---------------------------

  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);

    attribManager->compute_meshAttributs(mesh);

    bundler->compute_bundleByClass(mesh, 5);
    vector<float> Ib = bundler->get_Ib();
    vector<float> Ib_cosIt = bundler->get_Ib_cosIt();
    vector<float> Ib_R = bundler->get_Ib_dist();

    for(int j=0; j<Ib.size(); j++){
      I_all.push_back(Ib[j]);
      R_all.push_back(Ib_R[j]);
      cIt_all.push_back(Ib_cosIt[j]);
    }
  }

  //Write data on file
  ofstream file;
  file.open ("../data/data/IbyRbyIt.txt");
  file << "I "<<"R "<<"cIt"<<"\n";
  file << "--------------------------------"<<"\n";
  file <<std::fixed;
  for(int i=0; i<I_all.size(); i++){
    file <<setprecision(5)<< I_all[i]<<" "<<R_all[i]<<" "<<cIt_all[i]<<"\n";
  }
  file.close();

  //Say statistic data
  cout<< "Mean: " << fct_Mean(I_all) << " | CV: " << fct_CV(I_all) <<endl;

  //Plotting
  Gnuplot gp;
  gp << "set terminal wxt\n";
  gp << "set grid\n";
  gp << "set style line 11 lc rgb '#606060' lt 1\n";
  gp << "set border 3 back ls 11\n";
  gp << "set tics nomirror\n"; //Supress top and rigth tics
  gp << "set xlabel 'Distance R (m)'\n";
  gp << "set ylabel 'cos({/Symbol a})'\n";
  gp << "set zlabel 'Intensity [0;1]'\n";
  gp << "set zlabel rotate\n";
  gp << "set mouse\n";

  //Points
  gp << "splot '-' with points pt 7 ps 0.5 lc rgb 'black' notitle\n";
  gp.send1d(boost::make_tuple(R_all, cIt_all, I_all));
  //---------------------------
}
void Radiometry::plot_nDbycosIt(){
  if(!refManager->isref_Sphere()){
    refManager->load_SphereTarget_precomp();
  }
  list<Mesh*>* list = refManager->get_listSphere();
  vector<vector<float>> data_X, data_Y;
  //---------------------------

  //Bundle all reference targets
  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);

    bundler->compute_bundleByClass(mesh, 5);
    vector<float> Ib_cosIt = bundler->get_Ib_cosIt();
    vector<float> Ib_nD = bundler->get_Ib_nD();

    data_X.push_back(Ib_cosIt);
    data_Y.push_back(Normalize(Ib_nD));
  }

  //Compute classes
  bundler->compute_vectorByClass(data_X, data_Y);
  vector<float> data_Xb = bundler->get_vecb_dX();
  vector<float> data_Yb = bundler->get_vecb_dY();
  vector<float> err = bundler->get_vecb_std();

  //Plotting
  plotManager->set_namePlot("nDbyCosIt_Sphere");
  plotManager->set_Xlabel("{/Symbol a}");
  plotManager->set_Ylabel("Normalized size n_{k} of the k-th sample (k=1,..,90)");
  plotManager->set_Format_data1("pt 7 ps 0.3 lw 0.5 lc rgb 'black' notitle");
  plotManager->plot_CurveError(data_Xb, data_Yb, err);

  plotManager->set_Xlabel("{/Symbol a}");
  plotManager->set_Ylabel("Normalized size n_{k} of the k-th sample (k=1,..,90)");
  plotManager->set_Format_data1("pt 7 ps 0.3 lw 0.5 lc rgb 'black' notitle");
  plotManager->plot_Curve_Multi(data_X, data_Y);

  //---------------------------
  for(int i=0; i<data_Xb.size(); i++){
    console.AddLog("%.2f %.2f %.2f", data_Xb[i], data_Yb[i], err[i]);
  }
}
void Radiometry::plot_JbycosIt(){
  if(!refManager->isref_Sphere()){
    refManager->load_SphereTarget_precomp();
  }
  list<Mesh*>* list = refManager->get_listSphere();
  vector<vector<float>> data_X, data_Y;
  //---------------------------

  //Bundle all reference targets
  for(int i=0; i<list->size(); i++){
    Mesh* mesh = *next(list->begin(),i);

    bundler->compute_bundleByClass(mesh, 5);
    vector<float> Ib_cosIt = bundler->get_Ib_cosIt();
    vector<float> Ib_Jk = bundler->get_Ib_Jk();

    data_X.push_back(Ib_cosIt);
    data_Y.push_back(Ib_Jk);
  }

  //Compute classes
  bundler->compute_vectorByClass(data_X, data_Y);
  vector<float> Xd = bundler->get_vecb_dX();
  vector<float> Yd = bundler->get_vecb_dY();
  vector<float> err = bundler->get_vecb_std();

  //Plotting
  plotManager->set_namePlot("JbyCosIt_Sphere");
  plotManager->set_Xlabel("cos({/Symbol a})");
  plotManager->set_Ylabel("One half 95{/Symbol %} confidence interval J_{k}");
  plotManager->set_Format_data1("pt 7 ps 0.3 lw 0.5 lc rgb 'black' notitle");
  plotManager->plot_CurveError(Xd, Yd, err);

  plotManager->set_namePlot("JbyCosIt_Sphere");
  plotManager->set_Xlabel("cos({/Symbol a})");
  plotManager->set_Ylabel("One half 95{/Symbol %} confidence interval J_{k}");
  plotManager->plot_Curve_Multi(data_X, data_Y);

  //---------------------------
  for(int i=0; i<Xd.size(); i++){
    console.AddLog("%.2f %.2f %.2f", Xd[i], Yd[i], err[i]);
  }
}
void Radiometry::wrt_results(){
  ofstream corrResults;
  //---------------------------

  corrResults.open ("../../../ICorrection.txt");
  corrResults << "std(Is)"<<" "<<"std(Ic)"<<" "<<setprecision(3)<<"CV(Is)"<<" "<<"CV(Ic)"<<"\n";
  corrResults << "--------------------------------"<<"\n";
  for(int i=0; i<Is_std.size(); i++){
    corrResults << Is_std[i]<<" "<<Ic_std[i]<<" "<<Is_CV[i]<<" "<<Ic_CV[i]<<"\n";
  }

  //---------------------------
  corrResults.close();
}
void Radiometry::plot_ParameterSpace(){
  vector<vec3> PS_Sphere;
  //---------------------------

  //Compute parameter space
  if(refopeManager->compute_ParameterSpace_Sphere(5)){
    PS_Sphere = refopeManager->get_ParameterSpace();
  }

  //Plotting options
  plotManager->set_namePlot("3Dmap");
  plotManager->set_Xlabel("Distance R (m)");
  plotManager->set_Ylabel("cos({/Symbol a})");
  plotManager->set_Zlabel("Intensity [0;1]");
  plotManager->set_Format_data1("with points pt 7 ps 0.5 lc rgb 'black' title 'Raw data'");
  plotManager->plot_3Dmap_vec3(PS_Sphere);

  //---------------------------
}
