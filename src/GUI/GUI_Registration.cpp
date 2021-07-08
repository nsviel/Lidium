#include "GUI_Registration.h"

#include "../Engine/Scene.h"
#include "../Engine/Engine.h"
#include "../Registration/Pipeline/ICP.h"
#include "../Registration/Registration.h"
#include "../Operation/Transforms.h"
#include "../Engine/Glyphs.h"

//Constructor / Destructor
GUI_registration::GUI_registration(Engine* renderer){
  this->engineManager = renderer;
  //---------------------------

  this->sceneManager = engineManager->get_SceneManager();
  this->regisManager = engineManager->get_regisManager();
  this->icpManager = regisManager->get_icpManager();
  this->glyphManager = engineManager->get_glyphManager();
  this->regisManager = engineManager->get_regisManager();
  this->transformManager = new Transforms();

  this->regis_algo = 0;

  //---------------------------
}
GUI_registration::~GUI_registration(){}

//Main function
void GUI_registration::design_Registration(){
  //---------------------------

  this->regist_Registration();
  this->regist_Color();
  this->regist_Parameters();
  this->regist_DOF();
  this->regist_Stats();

  //---------------------------
}

//Subfunctions
void GUI_registration::regist_Color(){
  if(ImGui::CollapsingHeader("Colorization") && sceneManager->is_atLeastOneMesh()){
    int* colorMeth = regisManager->get_colorMethode();
    Mesh* mesh_P = sceneManager->get_selectedMesh();
    Mesh* mesh_Q = sceneManager->get_otherMesh();
    //---------------------------

    if(ImGui::RadioButton("None", colorMeth, 0)){
      regisManager->colorization(mesh_P, mesh_Q);
    }

    if(ImGui::RadioButton("Red Green", colorMeth, 1)){
      regisManager->colorization(mesh_P, mesh_Q);
    }

    if(ImGui::RadioButton("Red Green -", colorMeth, 2)){
      regisManager->colorization(mesh_P, mesh_Q);
    }

    if(ImGui::RadioButton("Transparent", colorMeth, 3)){
      regisManager->colorization(mesh_P, mesh_Q);
    }

    //---------------------------
    ImGui::Separator();
  }
}
void GUI_registration::regist_DOF(){
  if(ImGui::CollapsingHeader("DoF")){
    //---------------------------

    //Degree of freedoms - Rotation
    static bool Rx = true;
    static bool Ry = true;
    static bool Rz = true;
    if(ImGui::Checkbox("Rx", &Rx)){
      icpManager->set_dof_rotation(Rx, Ry, Rz);
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Ry", &Ry)){
      icpManager->set_dof_rotation(Rx, Ry, Rz);
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("Rz", &Rz)){
      icpManager->set_dof_rotation(Rx, Ry, Rz);
    }
    static float Rx_weight = 1;
    static float Ry_weight = 1;
    static float Rz_weight = 1;
    ImGui::PushItemWidth(40);
    if(ImGui::DragFloat("##555", &Rx_weight, 0.01f, 0, 1, "%.2f")){
      icpManager->set_dof_rot_weight(vec3(Rx_weight, Ry_weight, Rz_weight));
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(40);
    if(ImGui::DragFloat("##556", &Ry_weight, 0.01f, 0, 1, "%.2f")){
      icpManager->set_dof_rot_weight(vec3(Rx_weight, Ry_weight, Rz_weight));
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(40);
    if(ImGui::DragFloat("##557", &Rz_weight, 0.01f, 0, 1, "%.2f")){
      icpManager->set_dof_rot_weight(vec3(Rx_weight, Ry_weight, Rz_weight));
    }

    //Degree of freedoms - Translation
    static bool tx = true;
    static bool ty = true;
    static bool tz = true;
    if(ImGui::Checkbox("tx", &tx)){
      icpManager->set_dof_translation(tx, ty, tz);
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("ty", &ty)){
      icpManager->set_dof_translation(tx, ty, tz);
    }
    ImGui::SameLine();
    if(ImGui::Checkbox("tz", &tz)){
      icpManager->set_dof_translation(tx, ty, tz);
    }
    static float Tx_weight = 1;
    static float Ty_weight = 1;
    static float Tz_weight = 1;
    ImGui::PushItemWidth(40);
    if(ImGui::DragFloat("##558", &Tx_weight, 0.01f, 0, 1, "%.2f")){
      icpManager->set_dof_tra_weight(vec3(Tx_weight, Ty_weight, Tz_weight));
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(40);
    if(ImGui::DragFloat("##559", &Ty_weight, 0.01f, 0, 1, "%.2f")){
      icpManager->set_dof_tra_weight(vec3(Tx_weight, Ty_weight, Tz_weight));
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(40);
    if(ImGui::DragFloat("##560", &Tz_weight, 0.01f, 0, 1, "%.2f")){
      icpManager->set_dof_tra_weight(vec3(Tx_weight, Ty_weight, Tz_weight));
    }

    //---------------------------
    ImGui::Separator();
  }
}
void GUI_registration::regist_Parameters(){
  if(ImGui::CollapsingHeader("Parameters")){
    //---------------------------

    //Choose registration algo
    const char* cstch = "Custom\0ICP pcl\0NDT pcl\0GICP pcl\0LUM\0pts4congruents\0";
    ImGui::SetNextItemWidth(150);
    ImGui::Combo("##1", &regis_algo, cstch);

    //Choose optimization algo
    int* optim_algo = icpManager->get_optimizationMethod();
    const char* optim_choices = "Newton-Raphson\0SVD pcl\0LM pcl\0Dual quaternion pcl\0None\0";
    ImGui::SetNextItemWidth(150);
    ImGui::Combo("##2", optim_algo, optim_choices);

    //Choose center of mass - root or COM
    if(regis_algo == 0){
      int* com = icpManager->get_optimCOM();
      ImGui::RadioButton("COM", com, 0);
      ImGui::SameLine();
      ImGui::RadioButton("Root", com, 1);
    }

    //PNumber of iteration
    int* nbIter = regisManager->get_nbIter_max();
    ImGui::PushItemWidth(100);
    ImGui::InputInt("max iter", nbIter);

    float* SSE_max = regisManager->get_SSE_max();
    ImGui::PushItemWidth(100);
    ImGui::DragFloat("max SSE", SSE_max, 0.001f, 0, 1.0f, "%.4f");

    static float icpGain = 1.0f;
    ImGui::PushItemWidth(100);
    if(ImGui::DragFloat("icp gain", &icpGain, 0.01f, 0, 10, "%.2f")){
      icpManager->set_icpGain(icpGain);
    }

    //---------------------------
    ImGui::Separator();
  }
}
void GUI_registration::regist_Registration(){
  //---------------------------

  //Make an iteration
  ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(46, 75, 133, 255));
  static bool ICP_continu = false;
  if(ImGui::Button("Iteration+1", ImVec2(150,0)) || ICP_continu){
    if(sceneManager->is_atLeastMinNbMesh(2)){
      switch (regis_algo){
        case 0: regisManager->make_Iteration(); break;
        case 1: regisManager->make_algoPCL(0); break;
        case 2: regisManager->make_algoPCL(1); break;
        case 3: regisManager->make_algoPCL(2); break;
        case 4: regisManager->make_algoPCL(3); break;
        case 5: regisManager->make_algoPCL(4); break;
      }
    }
  }
  ImGui::PopStyleColor(1);

  //Start / stop continuous iterations
  if(ImGui::Button("start", ImVec2(70,0))){
    ICP_continu = true;
  }
  ImGui::SameLine();
  if(ImGui::Button("stop", ImVec2(70,0))){
    regisManager->restart();
    icpManager->reset();
    ICP_continu = false;
  }

  //Print error minimization
  if(ImGui::Button("Print", ImVec2(70,0))){
    regisManager->print_ICP();
  }

  //---------------------------
  ImGui::Separator();
}
void GUI_registration::regist_Stats(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  Mesh* mesh_Q = sceneManager->get_otherMesh();
  //---------------------------

  //DATA
  //--->translation
  vec3 Xt = icpManager->get_translat();
  ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "Xt : %.4f %.4f %.4f", Xt.x, Xt.y, Xt.z);
  vec3 Xr = icpManager->get_rotation();
  Xr.x = Xr.x *180/M_PI;
  Xr.y = Xr.y *180/M_PI;
  Xr.z = Xr.z *180/M_PI;
  ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "Xr(°) : %.4f %.4f %.4f", Xr.x, Xr.y, Xr.z);

  //--->world transformation
  vec3 E_trans, E_angle;
  if(sceneManager->is_atLeastOneMesh()){
    E_trans = transformManager->compute_translationsError(mesh);
    E_angle = transformManager->compute_anglesError(mesh);

    E_trans.x = E_trans.x * 1000;
    E_trans.y = E_trans.y * 1000;
    E_trans.z = E_trans.z * 1000;

    E_angle.x = E_angle.x * (180.0/3.141592653589793238463);
    E_angle.y = E_angle.y * (180.0/3.141592653589793238463);
    E_angle.z = E_angle.z * (180.0/3.141592653589793238463);

    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "Et (mm) : %.1f %.1f %.1f", E_trans.x, E_trans.y, E_trans.z);
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "Er (°) : %.3f %.3f %.3f", E_angle.x, E_angle.y, E_angle.z);
    ImGui::SameLine();
    if(ImGui::TreeNode("##146") && sceneManager->is_atLeastOneMesh()){
      mat4 matReal = mesh->transformation.RealTransformation;
      mat4 matIcp = mesh->transformation.TransformationMatrix;
      vec3 angleReal = transformManager->compute_anglesFromTransformationMatrix(matReal);
      vec3 transReal = transformManager->compute_translFromTransformationMatrix(matReal);
      vec3 angleIcp = transformManager->compute_anglesFromTransformationMatrix(matIcp);
      vec3 transIcp = transformManager->compute_translFromTransformationMatrix(matIcp);
      ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Real");
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(0.0f,1.0f,0.0f,1.0f), "Tw : %.3f %.3f %.3f", transReal.x, transReal.y, transReal.z);
      ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"ICP ");
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f,1.0f,1.0f,1.0f), "Tw : %.3f %.3f %.3f", transIcp.x, transIcp.y, transIcp.z);
      ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Real");
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(0.0f,1.0f,0.0f,1.0f), "Rw : %.3f %.3f %.3f", angleReal.x, angleReal.y, angleReal.z);
      ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"ICP ");
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f,1.0f,1.0f,1.0f), "Rw : %.3f %.3f %.3f", angleIcp.x, angleIcp.y, angleIcp.z);
    }

    //--->Iteration
    int iter = regisManager->get_iter();
    int* iter_max = regisManager->get_nbIter_max();
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "Iter : %d", iter);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "/%d", *iter_max);

    //--->Errors
    float SSE = icpManager->compute_SSE(mesh, mesh_Q);
    float* SSE_max = regisManager->get_SSE_max();
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "SSE : %f", SSE);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "/%.3f", *SSE_max);
    float MSE_groundTruth = icpManager->compute_MSE_groundTruth(mesh);
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "MSE ground truth : %f", MSE_groundTruth);
    float RMSE_groundTruth = icpManager->compute_RMSE_groundTruth(mesh);
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "RMSE ground truth : %f", RMSE_groundTruth);
    float MAE_groundTruth = icpManager->compute_MAE_groundTruth(mesh);
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "MAE ground truth : %f", MAE_groundTruth);

    //--->Time execution
    float ICP_duration = regisManager->get_duration();
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "ICP : %f", ICP_duration);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "ms");
    ImGui::SameLine();

    //--->Time execution more precise
    if(ImGui::TreeNode("##147")){
      ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "   Matching : %f", icpManager->get_time_matching());
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "ms");
      ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "   Rejection : %f", icpManager->get_time_rejection());
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "ms");
      ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "   Optimization : %f", icpManager->get_time_optimization());
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "ms");
    }
  }

  //---------------------------
}
