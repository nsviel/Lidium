#include "GUI_windows.h"

#include "../Engine/Engine.h"
#include "../Engine/Scene.h"
#include "../Operation/Operation.h"
#include "../Operation/Plotting.h"
#include "../Engine/OpenGL/Camera.h"
#include "../Operation/Functions/Extraction.h"
#include "../Operation/Functions/Heatmap.h"
#include "../Operation/Filter.h"
#include "../Operation/Functions/Selection.h"
#include "../Operation/Optimization/Polyfit.h"
#include "../Operation/Optimization/Fitting.h"
#include "../Engine/Glyphs.h"
#include "../Operation/Attribut.h"
#include "../Radiometry/Radiometry.h"


//Constructor / Destructor
GUI_windows::GUI_windows(Engine* engine){
  engineManager = engine;
  //---------------------------

  this->cameraManager = engineManager->get_CameraManager();
  this->sceneManager = engineManager->get_SceneManager();
  this->heatmapManager = engineManager->get_heatmapManager();
  this->filterManager = engineManager->get_FilterManager();
  this->selectionManager = engineManager->get_selectionManager();
  this->glyphManager = engineManager->get_glyphManager();
  this->attribManager = engineManager->get_AttribManager();
  this->radioManager = engineManager->get_RadioManager();
  this->opeManager = engineManager->get_OpeManager();
  this->fitManager = new Fitting(sceneManager);
  this->extractionManager = new Extraction(sceneManager);
  this->plotManager = new Plotting();

  //---------------------------
  this->init();
}
GUI_windows::~GUI_windows(){}

//Main functions
void GUI_windows::init(){
  //---------------------------

  this->show_asciiData = false;
  this->show_openOptions = false;
  this->show_camera = false;
  this->show_modifyFileInfo = false;
  this->show_saveOptions = false;
  this->show_transformation = false;
  this->show_extractCloud = false;
  this->show_cutCloud = false;
  this->show_filtering = false;
  this->show_normal = false;
  this->show_intensity = false;
  this->show_color = false;
  this->show_dataOpe = false;
  this->show_selection = false;
  this->show_fitting = false;
  this->show_heatmap = false;

  this->cloud_movement = configuration.CLOUD_movement;

  //---------------------------
}
void GUI_windows::window_Draw(){
  //---------------------------

  this->window_loadOption();
  this->window_camera();
  this->window_heatmap();
  this->window_saveOption();
  this->window_modifyFileInfo();
  this->window_asciiData();
  this->window_transformation();
  this->window_selection();
  this->window_extractCloud();
  this->window_cutCloud();
  this->window_filter();
  this->window_normal();
  this->window_intensity();
  this->window_color();
  this->window_dataOpe();
  this->window_fitting();

  //---------------------------
}

//General windows
void GUI_windows::window_loadOption(){
  if(show_openOptions){
    ImGui::Begin("Loading option", &show_openOptions, ImGuiWindowFlags_AlwaysAutoResize);
    //---------------------------

    //File format
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"File format");
    static int format = 3;
    ImGui::RadioButton("pts", &format, 0);
    ImGui::SameLine();
    ImGui::RadioButton("ptx", &format, 1);

    //Data to  retrieve
    static bool IdataON = true;
    if(ImGui::Checkbox("intensity", &IdataON)){
      loaderManager.load_option(0, 0, IdataON);
    }
    ImGui::SameLine();
    static bool RGBdataON = true;
    if(ImGui::Checkbox("color", &RGBdataON)){
      loaderManager.load_option(0, 1, RGBdataON);
    }
    ImGui::SameLine();
    static bool NdataON = false;
    if(ImGui::Checkbox("normal", &NdataON)){
      loaderManager.load_option(0, 2, NdataON);
    }
    ImGui::Separator();

    //Intensity data format
    if(IdataON){
      ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Intensity scaling");
      static int Idata = 2;
      static bool osef = false;
      if(ImGui::RadioButton("[0;1]", &Idata, 0)){
        loaderManager.load_option(0, 3, osef);
      }
      ImGui::SameLine();
      if(ImGui::RadioButton("[0;255]", &Idata, 1)){
        loaderManager.load_option(0, 4, osef);
      }
      ImGui::SameLine();
      if(ImGui::RadioButton("[-2048;2048]", &Idata, 2)){
        loaderManager.load_option(0, 5, osef);
      }
      ImGui::Separator();
    }

    //PTX
    if(format == 1){
      ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"ptx file format options");
      static int ptxMode = 1;
      if(ImGui::RadioButton("Scanner at origin", &ptxMode, 0)){
        loaderManager.load_option(2, 0, true);
        loaderManager.load_option(2, 1, false);
        loaderManager.load_option(2, 2, false);
      }
      static bool cloudTransfo = false;
      if(ImGui::RadioButton("Apply cloud transformation", &ptxMode, 1)){
        loaderManager.load_option(2, 0, false);
        loaderManager.load_option(2, 1, true);
        loaderManager.load_option(2, 2, false);
      }
      static bool separateClouds = true;
      if(ImGui::RadioButton("Separate clouds", &ptxMode, 2)){
        loaderManager.load_option(2, 0, false);
        loaderManager.load_option(2, 1, false);
        loaderManager.load_option(2, 2, true);
      }
      static bool notUseZValue = false;
      if(ImGui::Checkbox("Not use Z value", &notUseZValue)){
        loaderManager.load_option(2, 3, notUseZValue);
      }
      ImGui::Separator();
    }

    //---------------------------
    if(ImGui::Button("Load")){
      opeManager->loading();
    }
    if(ImGui::Button("Close")){
      show_openOptions = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_saveOption(){
  if(show_saveOptions){
    ImGui::Begin("Save", &show_saveOptions,ImGuiWindowFlags_AlwaysAutoResize);
    static bool all = false;
    //---------------------------

    /*ImGui::Text("Intensity scale");
    static int e = 3;
    if(ImGui::RadioButton("[0;1]", &e, 1)){
      loaderManager.save_option(e);
    }
    if(ImGui::RadioButton("[0;255]", &e, 2)){
      loaderManager.save_option(e);
    }
    if(ImGui::RadioButton("[-2048;2048]", &e, 3)){
      loaderManager.save_option(e);
    }*/

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Save")){
      opeManager->saving();
      show_saveOptions = false;
    }
    ImGui::SameLine();
    if(ImGui::Button("Save all")){
      opeManager->allSaving();
      show_saveOptions = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_asciiData(){
  if(show_asciiData){
    ImGui::Begin("Data", &show_asciiData);
    Mesh* mesh = sceneManager->get_selectedMesh();
    vector<vec3>& XYZ = mesh->location.OBJ;
    vector<vec4>& RGB = mesh->color.OBJ;
    vector<float>& Is = mesh->intensity.OBJ;
    vector<vec3>& Nxyz = mesh->normal.OBJ;
    //---------------------------

    //Settings
    static int nbLines = 100;
    ImGui::SliderInt("Number of lines", &nbLines, 1, mesh->NbPoints);

    int nb = 2;
    if(nbLines > XYZ.size()) nbLines = XYZ.size();
    if(mesh->intensity.hasData) nb++;
    if(mesh->color.hasData) nb++;
    if(mesh->normal.hasData) nb++;

    //Columns
    ImGui::Columns(nb);
    ImGui::Separator();
    ImGui::Text("#");
    ImGui::NextColumn();
    ImGui::Text("XYZ");
    ImGui::NextColumn();
    if(mesh->intensity.hasData){
      ImGui::Text("I");
      ImGui::NextColumn();
    }
    if(mesh->color.hasData){
      ImGui::Text("RGB");
      ImGui::NextColumn();
    }
    if(mesh->normal.hasData){
      ImGui::Text("Nxyz");
      ImGui::NextColumn();
    }
    ImGui::Separator();

    //Data in columns
    static int selected = -1;
    for(int i=0; i<nbLines; i++){
      ImGui::TextColored(ImVec4(0.4f,0.9f,0.4f,1.0f),"%i", i+1);
      ImGui::NextColumn();

      ImGui::Text("%f %f %f", XYZ[i].x, XYZ[i].y, XYZ[i].z);
      ImGui::NextColumn();

      if(mesh->intensity.hasData){
        ImGui::Text("%f", Is[i]);
        ImGui::NextColumn();
      }

      if(mesh->color.hasData){
        ImGui::Text("%f %f %f", RGB[i].x, RGB[i].y, RGB[i].z);
        ImGui::NextColumn();
      }

      if(mesh->normal.hasData){
        ImGui::Text("%f %f %f", Nxyz[i].x, Nxyz[i].y, Nxyz[i].z);
        ImGui::NextColumn();
      }
    }
    ImGui::Columns(1);
    ImGui::TreePop();

    //---------------------------
    ImGui::End();
  }
}
void GUI_windows::window_camera(){
  if(show_camera){
    ImGui::Begin(ICON_FA_CAMERA " Camera", &show_camera,ImGuiWindowFlags_AlwaysAutoResize);
    //---------------------------

    //Zoom - Field Of View
    float fov_value = cameraManager->get_fov();
    if(ImGui::SliderFloat("FOV (°)", &fov_value, 100.0f, 1.0f)){
      cameraManager->set_fov(fov_value);
    }
    static float cam_speed = configuration.CAM_MoveSpeed;
    if(ImGui::DragFloat("speed (m/s)", &cam_speed, 0.01, 0, 20, "%.2f")){
      cameraManager->set_cameraSpeed(cam_speed);
    }
    ImGui::Separator();

    //Camera mode
    static int projView;
    ImGui::Text("View");
    if(ImGui::RadioButton("Perspective", &projView, 0)){
      cameraManager->input_projView(projView);
    }
    if(ImGui::RadioButton("Orthographic", &projView, 1)){
      cameraManager->input_projView(projView);
    }
    if(ImGui::RadioButton("Top-View", &projView, 2)){
      cameraManager->input_projView(projView);
    }
    if(ImGui::RadioButton("Side-View", &projView, 3)){
      cameraManager->input_projView(projView);
    }
    ImGui::Separator();

    //Camera projection matrix
    glm::mat4 cam_pos = cameraManager->compute_worldPose();
    ImGui::Text("Model-View matrix");
    ImGui::SameLine();
    if(ImGui::Button("Print")){
      sayMat4(cam_pos);
    }
    ImGui::Columns(4, "Proj");
    for(int i=0;i<4;i++){
      ImGui::Separator();
      for(int j=0;j<4;j++){
        ImGui::Text("%.3f", cam_pos[i][j]);
        ImGui::NextColumn();
      }
    }
    ImGui::Separator();
    ImGui::Columns(1);

    //Camera position
    vec3* cam_position = cameraManager->get_camPosPtr();
    float *floatArray = &cam_position[0].x;
    if(ImGui::Button("R")){
      *cam_position = vec3(0,0,0);
    }
    ImGui::SameLine();
    ImGui::DragFloat3("Pos", floatArray, 0.01f, -100.0f, 100.0f);

    //Camera angles
    float HAngle = cameraManager->get_horizAngle();
    float VAngle = cameraManager->get_vertiAngle();
    ImGui::Text("Horizontal angle : %.2f°", HAngle * 180 / M_PI);
    ImGui::Text("Vertical angle : %.2f°", VAngle * 180 / M_PI);

    //Insert pre-defined pose
    /*static bool dPoseInit = false;
    if(ImGui::Button("Insert")){
      string zenity = "zenity --file-selection --title=CameraPose 2> /dev/null";
      FILE *file = popen(zenity.c_str(), "r");
      char filename[1024];
      const char* path_char = fgets(filename, 1024, file);
      mat4 dP;

      //Check if not empty
      if ((path_char != NULL) && (path_char[0] != '\0')){
        string path_str(path_char);
        path_str = path_str.substr(0, path_str.find('\n'));
        std::ifstream infile(path_str);
        std::string line;
        float a, b ,c, d;
        int cpt = 0;

        while (std::getline(infile, line))
        {
          std::istringstream iss(line);
          iss >> a >> b >> c >> d;

          dP[cpt][0] = a;
          dP[cpt][1] = b;
          dP[cpt][2] = c;
          dP[cpt][3] = d;

          cpt++;
        }

        mat4 orientation(
           dP[0][0], dP[0][1], dP[0][2], 0,
           dP[1][0], dP[1][1], dP[1][2], 0,
           dP[2][0], dP[2][1], dP[2][2], 0,
             0,       0,       0,     1);

        mat4 translation(
                  1,       0,       0, 0,
                  0,       1,       0, 0,
                  0,       0,       1, 0,
            -dP[0][3], -dP[1][3], -dP[2][3], 1);

        mat4 mvMatrix = orientation * translation;
        vec3 camPos = vec3(dP[0][3], dP[1][3], dP[2][3]);
        cameraManager->set_camPos(camPos);
        cameraManager->set_desiredViewMatrix(mvMatrix);

        dPoseInit = true;
        cameraManager->set_desiredPoseON(dPoseInit);
      }
    }
    ImGui::SameLine();
    static bool blockPose = false;
    if(ImGui::Checkbox("Block pose", &blockPose) && dPoseInit == true){
      cameraManager->set_desiredPoseON(dPoseInit);
    }*/

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_camera = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_heatmap(){
  if(show_heatmap){
    ImGui::Begin(ICON_FA_EYE " Heatmap", &show_heatmap, ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    //Apply heatMap on one cloud
    if(ImGui::Button("Apply", ImVec2(75,0))){
      if(sceneManager->is_atLeastOneMesh()){
        heatmapManager->set_HeatMap(mesh);
      }
    }
    ImGui::SameLine();

    //Heatmap all clouds
    static bool heatAll = false;
    if(ImGui::Button("Apply all", ImVec2(75,0))){
      if(sceneManager->is_atLeastOneMesh()){
        heatAll = !heatAll;
        heatmapManager->set_HeatMap_all(heatAll);
      }
    }

    //Select heatmap channel
    static int style_idx = 0;
    ImGui::SetNextItemWidth(75);
    if (ImGui::Combo("##1", &style_idx, "Is\0dist\0cos(It)\0It\0")){
        heatmapManager->set_HeatMapField(style_idx);
    }
    ImGui::SameLine();

    //Normalize palette
    bool* normalizeON = heatmapManager->get_param_Normalized();
    ImGui::Checkbox("Normalized", normalizeON);

    //Display color palette
    if(ImGui::Button("Palette", ImVec2(75,0))){
      if(mesh->intensity.heatmap){
        heatmapManager->plot_colorPalette(mesh);
      }
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_heatmap = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_transformation(){
  if(show_transformation && sceneManager->is_atLeastOneMesh()){
    ImGui::Begin("Transformation", &show_transformation, ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    //Z scanner
    /*ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Cloud elevation");

    //One or all cloud to operate
    static bool allClouds = false;
    ImGui::Checkbox("All clouds", &allClouds);

    static float Z_approx = 0.0f;
    static float Zpos = 0.0f;
    ImGui::PushItemWidth(75);
    if(ImGui::DragFloat("Z", &Zpos, 0.01f)){
      if(sceneManager->is_atLeastOneMesh()){
        transformManager.make_elevation(mesh, Zpos);
        sceneManager->update_CloudPosition(mesh);
        Z_approx = transformManager.fct_soilDetermination(mesh);
      }
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f,1.0f,0.0f,1.0f), "Z : %.3f", Z_approx);

    //Scanner height from ground
    static float Z_scan = 0.0f;
    if(ImGui::DragFloat("Scanner height", &Z_scan, 0.05f)){
      if(sceneManager->is_atLeastOneMesh()){
        Z_approx = transformManager.fct_soilDetermination(mesh);
      }
    }
    ImGui::SameLine();
    static int soilNbPoints = 10000;
    if(ImGui::DragInt("Ground pts", &soilNbPoints, 100)){
      if(sceneManager->is_atLeastOneMesh()){
        transformManager.set_soilNbPoints(soilNbPoints);
        Z_approx = transformManager.fct_soilDetermination(mesh);
      }
    }

    if(ImGui::Button("Accept##0")){
      if(allClouds){
        list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
        for(int i=0;i<list_Mesh->size();i++){
          Mesh* mesh = *next(list_Mesh->begin(),i);
          transformManager.fct_adjustPosToScanner(mesh, Z_scan);
          sceneManager->update_CloudPosition(mesh);
        }
      }
      else{
        if(sceneManager->is_atLeastOneMesh()){
          transformManager.fct_adjustPosToScanner(mesh, Z_scan);
          sceneManager->update_CloudPosition(mesh);
          Z_approx = transformManager.fct_soilDetermination(mesh);
        }
      }
    }
    ImGui::Separator();*/
    //---------------------------

    //Transformation matrix from initial
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Actual transformation from initial position");
    MatrixXf meshTransformation = glm_to_eigen_mat4(mesh->transformation.TransformationMatrix);
    std::stringstream ss;
    ss << meshTransformation;
    string bla = ss.str();
    static char meshTransformation_c[1024 * 16];
    strcpy(meshTransformation_c, bla.c_str());
    static ImGuiInputTextFlags flags = ImGuiInputTextFlags_AllowTabInput | ImGuiInputTextFlags_CtrlEnterForNewLine;
    ImGui::InputTextMultiline("##source", meshTransformation_c, IM_ARRAYSIZE(meshTransformation_c), ImVec2(400, ImGui::GetTextLineHeight() * 6), flags);

    //Real transformation matrix registration
    if(ImGui::CollapsingHeader("Real transformation matrix")){
      MatrixXf realTransformation = glm_to_eigen_mat4(mesh->transformation.RealTransformation);
      std::stringstream ss1;
      ss1 << realTransformation;
      string bla1 = ss1.str();
      static char realTransformation_c[1024 * 16];
      strcpy(realTransformation_c, bla1.c_str());
      ImGui::InputTextMultiline("##realTransfo", realTransformation_c, IM_ARRAYSIZE(realTransformation_c), ImVec2(400, ImGui::GetTextLineHeight() * 6), flags);

      if(ImGui::Button("Apply real transformation from init", ImVec2(300,0))){
        sceneManager->update_ResetMesh(mesh);
        transformManager.make_Transformation(mesh, vec3(0,0,0), mesh->transformation.RealTransformation);
        sceneManager->update_CloudPosition(mesh);
      }
    }
    ImGui::Separator();
    //---------------------------

    //Applicable transformation matrix
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Applicable transformation matrix");
    static char TransfoMatrix[1024 * 16] =
      "1.000 0.000 0.000 0.000\n"
      "0.000 1.000 0.000 0.000\n"
      "0.000 0.000 1.000 0.000\n"
      "0.000 0.000 0.000 1.000\n";

    if(ImGui::Button("Reset##2", ImVec2(100,0))){
      strcpy(TransfoMatrix,
          "1.000 0.000 0.000 0.000\n"
          "0.000 1.000 0.000 0.000\n"
          "0.000 0.000 1.000 0.000\n"
          "0.000 0.000 0.000 1.000\n");
    }
    ImGui::SameLine();
    if(ImGui::Button("Transpose", ImVec2(100,0))){
      //Convert char* to string
      string str(TransfoMatrix);

      //Convert string to MatrixXf
      istringstream iss(str);
      float m0, m1, m2, m3;
      float m4, m5, m6, m7;
      float m8, m9, m10, m11;
      float m12, m13, m14, m15;
      iss >> m0 >> m1 >> m2 >> m3;
      iss >> m4 >> m5 >> m6 >> m7;
      iss >> m8 >> m9 >> m10 >> m11;
      iss >> m12 >> m13 >> m14 >> m15;

      MatrixXf mat(4,4);
      mat(0,0) = m0; mat(0,1) = m1; mat(0,2) = m2; mat(0,3) = m3;
      mat(1,0) = m4; mat(1,1) = m5; mat(1,2) = m6; mat(1,3) = m7;
      mat(2,0) = m8; mat(2,1) = m9; mat(2,2) = m10; mat(2,3) = m11;
      mat(3,0) = m12; mat(3,1) = m13; mat(3,2) = m14; mat(3,3) = m15;
      MatrixXf mat2 = mat.transpose();

      //Copy result on the gui
      std::stringstream ss;
      ss << mat2;
      string bla = ss.str();
      strcpy(TransfoMatrix, bla.c_str());
    }

    ImGui::InputTextMultiline("##source2", TransfoMatrix, IM_ARRAYSIZE(TransfoMatrix), ImVec2(400, ImGui::GetTextLineHeight() * 6), flags);

    if(ImGui::Button("Apply from initial pos", ImVec2(150,0))){
      if(sceneManager->is_atLeastOneMesh()){
        mat4 mat = char_to_glm_mat4(TransfoMatrix);

        Mesh* mesh = sceneManager->get_selectedMesh();
        sceneManager->update_ResetMesh(mesh);
        transformManager.make_Transformation(mesh, vec3(0,0,0), mat);
        sceneManager->update_CloudPosition(mesh);
      }
    }
    ImGui::SameLine();
    if(ImGui::Button("Apply from actual pos", ImVec2(150,0))){
      if(sceneManager->is_atLeastOneMesh()){
        mat4 mat = char_to_glm_mat4(TransfoMatrix);

        //------------------
        Mesh* mesh = sceneManager->get_selectedMesh();
        transformManager.make_Transformation(mesh, vec3(0,0,0), mat);
        sceneManager->update_CloudPosition(mesh);
      }
    }
    if(ImGui::Button("Reverse from initial pos", ImVec2(150,0))){
      if(sceneManager->is_atLeastOneMesh()){
        mat4 mat = char_to_glm_mat4(TransfoMatrix);
        mat4 mat2 = inverse(mat);

        Mesh* mesh = sceneManager->get_selectedMesh();
        sceneManager->update_ResetMesh(mesh);
        transformManager.make_Transformation(mesh, vec3(0,0,0), mat);
        sceneManager->update_CloudPosition(mesh);
      }
    }
    ImGui::SameLine();
    if(ImGui::Button("Reverse from actual pos", ImVec2(150,0))){
      if(sceneManager->is_atLeastOneMesh()){
        mat4 mat = char_to_glm_mat4(TransfoMatrix);
        mat4 mat2 = inverse(mat);

        Mesh* mesh = sceneManager->get_selectedMesh();
        transformManager.make_Transformation(mesh, vec3(0,0,0), mat);
        sceneManager->update_CloudPosition(mesh);
      }
    }
    ImGui::Separator();

    //---------------------------
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Cloud translation");
    ImGui::PushItemWidth(150);
    static float trans[3] = {0.0f, 0.0f, 0.0f};
    ImGui::DragFloat3("XYZ", trans, 0.01f, -10.0f, 10.0f);
    ImGui::SameLine();
    if(ImGui::Button("Apply##1")){
      if(sceneManager->is_atLeastOneMesh()){
        vec3 translation = vec3(trans[0], trans[1], trans[2]);
        transformManager.make_translation(mesh, translation);
        sceneManager->update_CloudPosition(mesh);
        trans[0] = 0;
        trans[1] = 0;
        trans[2] = 0;
      }
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_transformation = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_filter(){
  if(show_filtering){
    ImGui::Begin("Filter", &show_filtering,ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    int sizeButton = 150;
    //---------------------------

    //Filter by angle
    static int maxAngle = 80;
    if(ImGui::Button("Filter by angle", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){

        list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
        for(int i=0; i<list_Mesh->size(); i++){
          Mesh* mesh = *next(list_Mesh->begin(),i);
          filterManager->filter_maxAngle(mesh, maxAngle);
          sceneManager->update_CloudPosition(mesh);
        }

      }
    }
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::DragInt("##1", &maxAngle, 1, 0, 90, "%d°");

    //Random sampling
    if(ImGui::Button("Random sampling", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        filterManager->sampling_random(mesh);
      }
    }
    ImGui::SameLine();
    static int percent = 50;
    ImGui::PushItemWidth(100);
    if(ImGui::DragInt("##2", &percent, 1, 0, 100, "%d%%")){
      filterManager->set_sampling(percent);
    }

    //Space sampling
    static float resolution = 0.1f;
    if(ImGui::Button("Space sampling", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        filterManager->sampling_spaceRadius(mesh, resolution);
      }
    }
    ImGui::SameLine();
    ImGui::DragFloat("##5", &resolution, 0.001, 0, 100, "%.4f");

    //Outliers filtering
    if(ImGui::Button("Outlier sampling", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        filterManager->sampling_outlier(mesh);
      }
    }
    ImGui::SameLine();
    static float radius = 0.1;
    ImGui::PushItemWidth(100);
    if(ImGui::DragFloat("##4", &radius, 0.0001, 0, 2, "%.5f")){
      filterManager->set_outRadiusSearch(radius);
    }

    //Statistical filtering
    if(ImGui::Button("Statistical sampling", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        filterManager->sampling_statistical(mesh);
      }
    }
    ImGui::SameLine();
    static float sampling_std = 1.0f;
    ImGui::PushItemWidth(100);
    if(ImGui::DragFloat("##5", &sampling_std, 0.0001, 0, 2, "%.5f")){
      filterManager->set_samplingstd(sampling_std);
    }

    //Sphere filtering
    if(ImGui::Button("Clean sphere cloud", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        filterManager->filter_sphereCleaning();
      }
    }
    ImGui::SameLine();
    static float sphereDiameter = 0.139f;
    ImGui::PushItemWidth(100);
    if(ImGui::DragFloat("##6", &sphereDiameter, 0.0001, 0, 2, "%.5f")){
      filterManager->set_sphereDiameter(sphereDiameter);
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_filtering = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_fitting(){
  if(show_fitting){
    ImGui::Begin("Fitting", &show_fitting,ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    int sizeButton = 150;
    //---------------------------

    //Sphere fitting
    if(ImGui::Button("Sphere fitting", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        fitManager->Sphere_MeshToCenter_all();
      }
    }

    //Plane fitting
    if(ImGui::Button("Plane fitting", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        fitManager->Plane_Mesh(mesh);
      }
    }

    //Axis alignement
    if(ImGui::Button("X alignement", ImVec2(sizeButton,0))){
      if(sceneManager->is_atLeastOneMesh()){
        transformManager.make_orientAxis_X(mesh);
        transformManager.make_alignAxis_X(mesh);
        sceneManager->update_CloudPosition(mesh);
      }
    }

    //---------------------------
    if(ImGui::Button("Close")){
      show_fitting = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_normal(){
  if(show_normal){
    ImGui::Begin("Attributs", &show_normal,ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    if(ImGui::Button("Compute attributs for all clouds", ImVec2(200,0))){
      attribManager->compute_meshAttributs_all();
    }

    //Standard normal computation
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Normals");

    static int normalMethod = 0;
    ImGui::PushItemWidth(207.5);
    ImGui::Combo("##11", &normalMethod, "PCL\0Hough\0Sphere fitting\0Plane fitting\0X axis\0Y axis\0Z axis\0");

    static float radius_normal = 0.03f;
    if(normalMethod == 1){
      ImGui::SetNextItemWidth(100);
      if(ImGui::InputFloat("Hough radius", &radius_normal, 0.01f, 1.0f, "%.3f")){
        if(radius_normal < 0.01) radius_normal = 0.01;
        attribManager->set_normalRadiusSeach(radius_normal);
      }
    }
    if(normalMethod == 2){
      static float radius = attribManager->get_sphereRadius();
      ImGui::SetNextItemWidth(100);
      if(ImGui::DragFloat("Sphere radius", &radius, 0.0001, 0, 1, "%.4f")){
        attribManager->set_sphereRadius(radius);
      }
    }

    if(ImGui::Button("Compute", ImVec2(200,0))){
      if(sceneManager->is_atLeastOneMesh()){
        //---------------------------

        if(normalMethod == 0){
          attribManager->compute_normals(mesh);
        }

        if(normalMethod == 1){
          attribManager->compute_normals_Hough(mesh);
        }

        if(normalMethod == 2){
          attribManager->compute_normals_sphere(mesh);
        }

        if(normalMethod == 3){
          attribManager->compute_normals_planFitting(mesh);
        }

        if(normalMethod == 4){
          float angle = transformManager.make_orientAxis_X(mesh);
          attribManager->compute_normals_planXaxis(mesh);
          vec3 rotation = vec3(0, 0, -angle);
          transformManager.make_rotation(mesh, vec3(0,0,0), rotation);
          mesh->normal.Initial = mesh->normal.OBJ;
          sceneManager->update_CloudPosition(mesh);
        }

        if(normalMethod == 5){
          attribManager->compute_normals_planYaxis(mesh);
          mesh->normal.Initial = mesh->normal.OBJ;
          sceneManager->update_CloudPosition(mesh);
        }

        if(normalMethod == 6){
          attribManager->compute_normals_planZaxis(mesh);
          mesh->normal.Initial = mesh->normal.OBJ;
          sceneManager->update_CloudPosition(mesh);
        }

        glyphManager->obj_normals(mesh);

        //---------------------------
      }
    }
    if(ImGui::Button("Compute all clouds", ImVec2(200,0))){
      if(sceneManager->is_atLeastOneMesh()){
        //---------------------------


        list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
        for(int i=0;i<list_Mesh->size();i++){
          Mesh* mesh = *next(list_Mesh->begin(),i);

          if(normalMethod == 0){
            attribManager->compute_normals(mesh);
          }

          if(normalMethod == 1){
            attribManager->compute_normals_Hough(mesh);
          }

          if(normalMethod == 2){
            attribManager->compute_normals_sphere(mesh);
          }

          if(normalMethod == 3){
            attribManager->compute_normals_planFitting(mesh);
          }

          if(normalMethod == 4){
            float angle = transformManager.make_orientAxis_X(mesh);
            attribManager->compute_normals_planXaxis(mesh);
            vec3 rotation = vec3(0, 0, -angle);
            transformManager.make_rotation(mesh, vec3(0,0,0), rotation);
            mesh->normal.Initial = mesh->normal.OBJ;
            sceneManager->update_CloudPosition(mesh);
          }

          if(normalMethod == 5){
            attribManager->compute_normals_planYaxis(mesh);
            mesh->normal.Initial = mesh->normal.OBJ;
            sceneManager->update_CloudPosition(mesh);
          }

          if(normalMethod == 6){
            attribManager->compute_normals_planZaxis(mesh);
            mesh->normal.Initial = mesh->normal.OBJ;
            sceneManager->update_CloudPosition(mesh);
          }

          glyphManager->obj_normals(mesh);
        }

        //---------------------------
      }
    }

    if(ImGui::Button("Reoriente to origin", ImVec2(200,0))){
      if(sceneManager->is_atLeastOneMesh()){
        attribManager->compute_normals_reorientToOrigin(mesh);
        glyphManager->obj_normals(mesh);
      }
    }
    if(ImGui::Button("Invert", ImVec2(200,0))){
      if(sceneManager->is_atLeastOneMesh()){
        attribManager->compute_normals_invert();
        glyphManager->obj_normals(mesh);
      }
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_normal = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_intensity(){
  if(show_intensity){
    ImGui::Begin("Intensity", &show_intensity, ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Intensity functions");

    //Display intensity or color channel
    static bool colorON = false;
    if(ImGui::Button("Intensity / Color all", ImVec2(200,0))){
      colorON = !colorON;
      if(colorON){
        attribManager->set_colorRGB_all();
      }else{
        attribManager->set_colorI_all();
      }
    }

    //Invert the intensity values
    if(ImGui::Button("Inversion Is", ImVec2(200,0))){
      if(sceneManager->is_atLeastOneMesh()){
        attribManager->compute_intensityInversion();
        sceneManager->update_IntensityToColor(mesh);
      }
    }

    //Normalize the intensity values
    if(ImGui::Button("Normalize Intensity to [0,1]", ImVec2(200,0))){
      vector<float>& Is = mesh->intensity.OBJ;
      Is = Normalize(Is);
      sceneManager->update_IntensityToColor(mesh);
    }

    //Intensity display slider
    ImGui::Text("Selection intensity");
    static float min = 0, max = 1;
    if(ImGui::DragFloatRange2("##123321", &min, &max, 0.001f, 0.00f, 1.0f, "%.3f", "%.3f")){
      mesh->intensity.OBJ = mesh->intensity.Initial;
      attribManager->fct_IsRange(vec2(min, max));
    }

    //Intensity shifting
    ImGui::Text("Shift intensity");
    static float shift = 0.01;
    ImGui::PushItemWidth(100);
    ImGui::InputFloat("##123322", &shift, 0.0f, 1.0f, "%.4f");
    ImGui::SameLine();
    ImGuiStyle& style = ImGui::GetStyle();
    float spacing = style.ItemInnerSpacing.x;
    ImGui::PushButtonRepeat(true);
    if(ImGui::ArrowButton("##left", ImGuiDir_Left)){
      vector<float>& Is = mesh->intensity.OBJ;
      for(int i=0; i<Is.size(); i++){
        Is[i] = Is[i] - shift;
        if(Is[i] < 0.0f) Is[i] = 0.0f;
      }
      sceneManager->update_IntensityToColor(mesh);
    }
    ImGui::SameLine(0.0f, spacing);
    if(ImGui::ArrowButton("##right", ImGuiDir_Right)){
      vector<float>& Is = mesh->intensity.OBJ;
      for(int i=0; i<Is.size(); i++){
        Is[i] = Is[i] + shift;
        if(Is[i] > 1.0f) Is[i] = 1.0f;
      }
      sceneManager->update_IntensityToColor(mesh);
    }
    ImGui::PopButtonRepeat();
    ImGui::Separator();

    //Reconvert intensity
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Intensity scaling");
    if(ImGui::Button("Restore I initial", ImVec2(200,0))){
      mesh->intensity.OBJ = mesh->intensity.Initial;
      sceneManager->update_IntensityToColor(mesh);
    }
    if(ImGui::Button("I:255->2048", ImVec2(100,0))){
      attribManager->fct_convert255to2048(mesh);
      sceneManager->update_IntensityToColor(mesh);
    }
    ImGui::SameLine();
    if(ImGui::Button("I:2048->255", ImVec2(100,0))){
      attribManager->fct_convert2048to255(mesh);
      sceneManager->update_IntensityToColor(mesh);
    }
    if(ImGui::Button("I:1->2048", ImVec2(100,0))){
      vector<float>& Is = mesh->intensity.OBJ;
      for(int i=0; i<Is.size(); i++){
        Is[i] = Is[i]*4096-2048;
      }
      sceneManager->update_IntensityToColor(mesh);
    }
    ImGui::SameLine();
    if(ImGui::Button("I:2048->1", ImVec2(100,0))){
      vector<float>& Is = mesh->intensity.OBJ;
      for(int i=0; i<Is.size(); i++){
        Is[i] = (Is[i]+2048)/4096;
      }
      sceneManager->update_IntensityToColor(mesh);
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_intensity = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_color(){
  if(show_color){
    ImGui::Begin("Color", &show_color, ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    //Color channel
    ImGui::Text("Color channel");
    static int e = 0;
    ImGui::Separator();
    ImGui::Columns(2);
    ImGui::Text("Selected cloud"); ImGui::NextColumn();
    ImGui::Text("All cloud"); ImGui::NextColumn();
    if(ImGui::RadioButton("I    ##1", &e, 1)){
      if(mesh->intensity.hasData){
        sceneManager->update_IntensityToColor(mesh);
      }
    } ImGui::NextColumn();
    if(ImGui::RadioButton("I    ##2", &e, 2)){
      list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
      for(int i=0;i<list_Mesh->size();i++){
        Mesh* mesh = *next(list_Mesh->begin(),i);

        if(mesh->intensity.hasData){
          sceneManager->update_IntensityToColor(mesh);
        }
      }
    } ImGui::NextColumn();
    if(ImGui::RadioButton("RGB  ##1", &e, 3)){
      if(mesh->color.hasData){
        mesh->color.OBJ = mesh->color.Initial;
        sceneManager->update_CloudColor(mesh);
      }
    } ImGui::NextColumn();
    if(ImGui::RadioButton("RGB  ##2", &e, 4)){
      list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
      for(int i=0;i<list_Mesh->size();i++){
        Mesh* mesh = *next(list_Mesh->begin(),i);

        if(mesh->color.hasData){
          mesh->color.OBJ = mesh->color.Initial;
          sceneManager->update_CloudColor(mesh);
        }
      }
    } ImGui::NextColumn();
    if(ImGui::RadioButton("RGB*I##1", &e, 5)){
      if(mesh->intensity.hasData && mesh->color.hasData){
        attribManager->set_enhancedColor(mesh);
      }else{
        cout<<"Selected mesh: I="<<mesh->intensity.hasData<<" | Color="<<mesh->color.hasData<<endl;
      }
    } ImGui::NextColumn();
    if(ImGui::RadioButton("RGB*I##2", &e, 6)){
      list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
      for(int i=0;i<list_Mesh->size();i++){
        Mesh* mesh = *next(list_Mesh->begin(),i);

        if(mesh->intensity.hasData && mesh->color.hasData){
          attribManager->set_enhancedColor(mesh);
        }
      }
    } ImGui::NextColumn();
    ImGui::Columns(1);
    ImGui::Separator();

    //Color functions
    if(ImGui::Button("Supress color all clouds", ImVec2(150,0))){
      list<Mesh*>* list_Mesh = sceneManager->get_listMesh();
      for(int i=0;i<list_Mesh->size();i++){
        Mesh* mesh = *next(list_Mesh->begin(),i);

        if(mesh->color.OBJ.size() != 0){
          mesh->color.OBJ.clear();
        }
        mesh->color.hasData = false;
        sceneManager->update_dataFormat(mesh);
      }
    }
    if(ImGui::Button("Random color for all points", ImVec2(150,0))){
      attribManager->set_randomizeAllPointsColor(mesh);
      sceneManager->update_CloudColor(mesh);
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_color = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_selection(){
  if(show_selection){
    ImGui::Begin("Selection part", &show_selection,ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    ImGui::Text("Point");
    static bool selectionPtON = false;
    if(ImGui::Checkbox("Selection mode", &selectionPtON)){
      if(sceneManager->is_atLeastOneMesh() && selectionPtON){
        heatmapManager->set_HeatMap_all(true);
        selectionManager->set_markMode("sphere");
        engineManager->set_pointSize(10);
        cloud_movement = false;
      }

      if(!selectionPtON){
        heatmapManager->set_HeatMap_all(false);
        selectionManager->set_markMode("cube");
        engineManager->set_pointSize(1);
        cloud_movement = true;
      }
    }
    static float sensibility = 0.005f;
    if(ImGui::DragFloat("Sensibility", &sensibility, 0.0001, 0, 1, "%.4f")){
      selectionManager->set_selectionSensibility(sensibility);
    }
    if(ImGui::Button("Supress all points", ImVec2(200,0))){
      selectionManager->mark_supressSelectedPoints_all();
      //heatmapManager->set_HeatMap_all(true);
    }
    ImGui::Separator();

    ImGui::Text("Cloud part");
    static float xmin = 0, xmax = 100;
    static float ymin = 0, ymax = 100;
    static float zmin = 0, zmax = 100;
    if(ImGui::Button("Reset", ImVec2(100,0)) || ImGui::IsKeyPressed(258)){
      xmin = 0; xmax = 100;
      ymin = 0; ymax = 100;
      zmin = 0; zmax = 100;
    }
    ImGui::SameLine();
    bool* highlightON = extractionManager->get_highlightON();
    if(ImGui::Checkbox("Hightligth", highlightON) || ImGui::IsKeyPressed(258)){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->fct_highlighting(mesh);
      }
    }

    //AABB manipulators
    ImGui::PushAllowKeyboardFocus(false);
    if(ImGui::DragFloatRange2("X", &xmin, &xmax, 0.25f, 0.01f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    if(ImGui::DragFloatRange2("Y", &ymin, &ymax, 0.25f, 0.0f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    if(ImGui::DragFloatRange2("Z", &zmin, &zmax, 0.25f, 0.0f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    ImGui::PopAllowKeyboardFocus();

    if(ImGui::Button("Extract part", ImVec2(100,0))){
      if(sceneManager->is_atLeastOneMesh()){
        vec3 min_pourc = vec3(xmin, ymin, zmin);
        vec3 max_pourc = vec3(xmax, ymax, zmax);
        extractionManager->fct_selectPart(mesh, min_pourc, max_pourc);
      }
    }

    //Table of selected parts
    list<subpart*>* list = extractionManager->get_listParts();
    ImGui::Columns(5, "part");
    ImGui::Separator();
    ImGui::Text("ID"); ImGui::NextColumn();
    ImGui::Text("Name"); ImGui::NextColumn();
    ImGui::Text("Cloud"); ImGui::NextColumn();
    ImGui::Text("Min"); ImGui::NextColumn();
    ImGui::Text("Max"); ImGui::NextColumn();
    ImGui::Separator();
    static int selected = -1;

    ImGui::SetColumnWidth(0,50);
    ImGui::SetColumnWidth(1,125);

    for (int i=0; i<list->size(); i++){
      subpart* part = *next(list->begin(),i);

      //ID
      char label[32];
      sprintf(label, "%04d", i);
      if(ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_None)){
        selected = i;
      }
      bool hovered = ImGui::IsItemHovered();
      ImGui::NextColumn();

      //Name
      static char str[256];
      strcpy(str, part->name.c_str());
      string truc = "##" + to_string(i);
      const char* ID_label = truc.c_str();
      if(ImGui::InputText(ID_label, str, IM_ARRAYSIZE(str))){
        part->name = str;
      }

      //Removal cross
      ImGui::PushID(i);
      ImGui::SameLine();
      if(ImGui::SmallButton("X")){
        extractionManager->supress_selectedpart(part);
      }
      ImGui::PopID();
      ImGui::NextColumn();

      //Cloud
      ImGui::Text("%s", part->namePC.c_str());
      ImGui::NextColumn();

      //Min
      vec3 minloc = part->minloc;
      ImGui::Text("%.1f %.1f %.1f", minloc.x, minloc.y, minloc.z);
      ImGui::NextColumn();

      //Max
      vec3 maxloc = part->maxloc;
      ImGui::Text("%.1f %.1f %.1f", maxloc.x, maxloc.y, maxloc.z);
      ImGui::NextColumn();
    }
    ImGui::Columns(1);
    ImGui::Separator();

    //---------------------------
    if(ImGui::Button("Close")){
      show_selection = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_extractCloud(){
  if(show_extractCloud){
    ImGui::Begin("Extract cloud", &show_extractCloud,ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    //Extraction functions
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Extract from AABB manipulators");
    bool* highlightON = extractionManager->get_highlightON();
    if(ImGui::Checkbox("Hightligth", highlightON)){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->fct_highlighting(mesh);
      }
    }

    //Reset manipulators
    static float xmin = 0, xmax = 100;
    static float ymin = 0, ymax = 100;
    static float zmin = 0, zmax = 100;
    if(ImGui::Button("Reset X Y Z", ImVec2(100,0))){
      xmin = 0;
      xmax = 100;
      ymin = 0;
      ymax = 100;
      zmin = 0;
      zmax = 100;
    }

    //AABB manipulators
    ImGui::PushAllowKeyboardFocus(false);
    if(ImGui::DragFloatRange2("X", &xmin, &xmax, 0.25f, 0.01f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    if(ImGui::DragFloatRange2("Y", &ymin, &ymax, 0.25f, 0.0f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    if(ImGui::DragFloatRange2("Z", &zmin, &zmax, 0.25f, 0.0f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    ImGui::PopAllowKeyboardFocus();

    //Extract a new cloud from AABB manipulators
    if(ImGui::Button("Extract cloud", ImVec2(100,0))){
      if(sceneManager->is_atLeastOneMesh()){
        //Reset color
        *highlightON = false;
        extractionManager->fct_highlighting(mesh);

        //Extract cloud
        extractionManager->fct_extractCloud(mesh);
      }
    }
    ImGui::SameLine();
    static bool sliceON = false;
    if(ImGui::Checkbox("Slice", &sliceON)){
      extractionManager->set_sliceON(sliceON);
    }
    ImGui::Separator();

    //Extract points selected with the mouse frame
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Extract from mouse frame");
    if(ImGui::Button("Extract selected frame", ImVec2(150,0))){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->fct_extractSelected(mesh);
      }
    }
    ImGui::Separator();

    //Merge and extract two clouds
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Merge and extract two clouds");
    if(ImGui::Button("Merge clouds", ImVec2(150,0))){
      if(sceneManager->get_listMeshSize() >= 2){
        Mesh* mesh_2 = sceneManager->get_otherMesh();
        extractionManager->fct_merging_newCloud(mesh, mesh_2);
      }
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_extractCloud = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_cutCloud(){
  if(show_cutCloud){
    ImGui::Begin("Cut cloud", &show_cutCloud,ImGuiWindowFlags_AlwaysAutoResize);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    bool* highlightON = extractionManager->get_highlightON();
    if(ImGui::Checkbox("Hightligth", highlightON) || ImGui::IsKeyPressed(258)){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->fct_highlighting(mesh);
      }
    }
    ImGui::SameLine();

    //Reset manipulator
    static float xmin = 0, xmax = 100;
    static float ymin = 0, ymax = 100;
    static float zmin = 0, zmax = 100;
    if(ImGui::Button("Reset X Y Z", ImVec2(100,0))){
      xmin = 0;
      xmax = 100;
      ymin = 0;
      ymax = 100;
      zmin = 0;
      zmax = 100;
    }

    //AABB manipulators
    ImGui::PushAllowKeyboardFocus(false);
    if(ImGui::DragFloatRange2("X", &xmin, &xmax, 0.25f, 0.01f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    if(ImGui::DragFloatRange2("Y", &ymin, &ymax, 0.25f, 0.0f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    if(ImGui::DragFloatRange2("Z", &zmin, &zmax, 0.25f, 0.0f, 100.0f, "%.1f %%", "%.1f %%")){
      if(sceneManager->is_atLeastOneMesh()){
        extractionManager->set_AABB_min(vec3(xmin,ymin,zmin));
        extractionManager->set_AABB_max(vec3(xmax,ymax,zmax));
        glyphManager->obj_aabb(mesh);
      }
    }
    ImGui::PopAllowKeyboardFocus();

    //Cuttinf functions
    if(ImGui::Button("Cut", ImVec2(100,0))){
      if(sceneManager->is_atLeastOneMesh()){
        //Reset color
        *highlightON = false;
        extractionManager->fct_highlighting(mesh);

        //Cut cloud
        extractionManager->fct_cutCloud(mesh);
      }
    }
    ImGui::SameLine();
    if(ImGui::Button("Cut all cloud", ImVec2(100,0))){
      if(sceneManager->is_atLeastOneMesh()){
        //Reset color
        *highlightON = false;
        extractionManager->fct_highlighting(mesh);

        //Cut clouds
        extractionManager->fct_cutCloud_all();
      }
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_cutCloud = false;
    }
    ImGui::End();
  }
}
void GUI_windows::window_dataOpe(){
  if(show_dataOpe){
    ImGui::Begin("Data", &show_dataOpe,ImGuiWindowFlags_AlwaysAutoResize);
    //---------------------------

    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Write on file");
    if(ImGui::Button("I-R-a all without ref", ImVec2(200,0))){
      list<Mesh*>* list_Mesh = sceneManager->get_listMesh();

      //Write data on file
      ofstream file;
      file.open ("../data/data_IRA_woRef.txt");
      file << "Name "<<"R "<<"alpha "<<"IRAWmean "<<"IRAWstd "<<"ICORmean "<<"ICORstd "<<"\n";
      file << "--------------------------------"<<"\n";

      for(int i=0;i<list_Mesh->size();i++){
        Mesh* mesh = *next(list_Mesh->begin(),i);

        if(mesh->Name.find("Sphere") == std::string::npos &&
        mesh->Name.find("Spectralon") == std::string::npos){
          if(mesh->attribut.dist.size() == 0) attribManager->compute_Distances(mesh);
          if(mesh->attribut.It.size() == 0) attribManager->compute_cosIt(mesh);

          float R = fct_Mean(mesh->attribut.dist);
          float A = fct_Mean(mesh->attribut.It);

          float Irm = fct_Mean(mesh->intensity.Initial);
          float Irs = fct_std(mesh->intensity.Initial);
          float Icm = fct_Mean(mesh->intensity.OBJ);
          float Ics = fct_std(mesh->intensity.OBJ);

          file <<std::fixed;
          file <<mesh->Name<<" ";
          file <<setprecision(3)<< R<<" "<<A<<" ";
          file <<setprecision(3)<< Irm<<" "<<Irs<<" ";
          file <<setprecision(3)<< Icm<<" "<<Ics<<"\n";
          //---------------------------
        }
      }
      file.close();
    }
    if(ImGui::Button("I all without ref", ImVec2(200,0))){
      list<Mesh*>* list_Mesh = sceneManager->get_listMesh();

      //Write data on file
      ofstream file;
      file.open ("../data/data_IRA_woRef.txt");
      file << "Imean "<<"Istd "<<"\n";
      file << "--------------------------------"<<"\n";

      for(int i=0;i<list_Mesh->size();i++){
        Mesh* mesh = *next(list_Mesh->begin(),i);

        if(mesh->Name.find("Sphere") == std::string::npos &&
        mesh->Name.find("Spectralon") == std::string::npos){

          float Im = fct_Mean(mesh->intensity.OBJ);
          float Is = fct_std(mesh->intensity.OBJ);

          file <<std::fixed;
          file <<setprecision(3)<< Im<<" "<<Is<<"\n";
          //---------------------------
        }
      }
      file.close();
    }
    if(ImGui::Button("R-Alpha all without ref", ImVec2(200,0))){
      list<Mesh*>* list_Mesh = sceneManager->get_listMesh();

      //Write data on file
      ofstream file;
      file.open ("../data/data_IRA_woRef.txt");
      file << "R "<<"Alpha "<<"\n";
      file << "--------------------------------"<<"\n";

      for(int i=0;i<list_Mesh->size();i++){
        Mesh* mesh = *next(list_Mesh->begin(),i);

        if(mesh->Name.find("Sphere") == std::string::npos &&
        mesh->Name.find("Spectralon") == std::string::npos){
          if(mesh->attribut.dist.size() == 0) attribManager->compute_Distances(mesh);
          if(mesh->attribut.It.size() == 0) attribManager->compute_cosIt(mesh);

          float R = fct_Mean(mesh->attribut.dist);
          float A = fct_Mean(mesh->attribut.It);

          file <<std::fixed;
          file <<setprecision(3)<< R<<" "<<A<<"\n";
          //---------------------------
        }
      }
      file.close();
    }
    if(ImGui::Button("Write data in file", ImVec2(200,0))){
      ofstream file;
      file.open ("../data/data_allCloud.txt");
      file << "Name R alpha I"<<"\n";
      file << "--------------------------------"<<"\n";
      //---------------------------

      list<Mesh*>* list = sceneManager->get_listMesh();
      for(int i=0; i<list->size(); i++){
        Mesh* mesh = *next(list->begin(),i);

        float R = fct_Mean(mesh->attribut.dist);
        float A = fct_Mean(mesh->attribut.It);
        float I = fct_Mean(mesh->intensity.OBJ);

        //file <<setprecision(5)<<mesh->Name<<" "<<R<<" "<<A<<" "<<I<<"\n";
        file <<setprecision(5)<<R<<" "<<A<<" "<<I<<"\n";
      }

      //---------------------------
      file.close();
    }
    if(ImGui::Button("Write data Spectralon", ImVec2(200,0))){
      radioManager->plot_SpectralonAllMeans();
    }
    if(ImGui::Button("Write intensity in file", ImVec2(200,0))){
      ofstream file;
      file.open ("../data/data_intensity.txt");

      Mesh* mesh = sceneManager->get_selectedMesh();
      vector<float>& Is = mesh->intensity.OBJ;
      //---------------------------

      for(int i=0; i<Is.size(); i++){
        file <<setprecision(4)<<Is[i]<<"\n";
      }

      //---------------------------
      file.close();
    }

    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Write on term");
    if(ImGui::Button("I Istd all", ImVec2(200,0))){
      //---------------------

      if(sceneManager->is_atLeastOneMesh()){
        list<Mesh*>* list = sceneManager->get_listMesh();

        //Intensity
        cout<<"-- Intensity --"<<endl;
        for(int i=0; i<list->size(); i++){
          Mesh* mesh = *next(list->begin(),i);

          cout<<std::fixed<<std::setprecision(2)<<"& "<<fct_Mean(mesh->intensity.OBJ)<<endl;
        }

        //Standard deviation
        cout<<"-- SD --"<<endl;
        for(int i=0; i<list->size(); i++){
          Mesh* mesh = *next(list->begin(),i);

          cout<<std::fixed<<std::setprecision(3)<<"& "<<fct_std(mesh->intensity.OBJ)<<endl;
        }
      }

      //---------------------
    }
    if(ImGui::Button("Min-Max R It all", ImVec2(200,0))){
      vector<float> min_R, min_It;
      vector<float> max_R, max_It;
      //---------------------

      if(sceneManager->is_atLeastOneMesh()){
        list<Mesh*>* list = sceneManager->get_listMesh();
        for(int i=0; i<list->size(); i++){
          Mesh* mesh = *next(list->begin(),i);

          vector<float>& dist = mesh->attribut.dist;
          if(dist.size() == 0) attribManager->compute_Distances(mesh);
          min_R.push_back(fct_min(dist));
          max_R.push_back(fct_max(dist));

          vector<float>& It = mesh->attribut.It;
          if(It.size() == 0) attribManager->compute_cosIt(mesh);
          min_It.push_back(fct_min(It));;
          max_It.push_back(fct_max(It));;
        }
      }

      //---------------------
      cout<<"R: Min = "<<fct_min(min_R)<<" | Max = "<<fct_max(max_R)<<endl;
      cout<<"It: Min = "<<fct_min(min_It)<<" | Max = "<<fct_max(max_It)<<endl;
    }
    if(ImGui::Button("R It all", ImVec2(200,0))){
      //---------------------

      if(sceneManager->is_atLeastOneMesh()){
        list<Mesh*>* list = sceneManager->get_listMesh();

        //Distances
        cout<<"-- Distances --"<<endl;
        for(int i=0; i<list->size(); i++){
          Mesh* mesh = *next(list->begin(),i);

          vector<float>& dist = mesh->attribut.dist;
          if(dist.size() == 0) attribManager->compute_Distances(mesh);

          cout<<std::fixed<<std::setprecision(2)<<"& "<<fct_Mean(dist)<<endl;
        }

        //Angle
        cout<<"-- Angles --"<<endl;
        for(int i=0; i<list->size(); i++){
          Mesh* mesh = *next(list->begin(),i);

          vector<float>& It = mesh->attribut.It;
          if(It.size() == 0) attribManager->compute_cosIt(mesh);

          cout<<std::fixed<<std::setprecision(2)<<"& "<<fct_Mean(It)<<endl;
        }

      }

      //---------------------
    }

    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Curve");
    if(ImGui::Button("nD by cos It", ImVec2(200,0))){
      radioManager->plot_nDbycosIt();
    }
    if(ImGui::Button("Jk by cos It", ImVec2(200,0))){
      radioManager->plot_JbycosIt();
    }
    if(ImGui::Button("Test regression", ImVec2(200,0))){
      vector<float> X{
        1.30806,
        2.29148,
        3.28815,
        4.2841,
        5.29482,
        7.32926,
        9.30038,
        11.3138,
        13.3208,
        15.3293,
        17.3446,
        19.3401,
        21.3557,
        23.3466,
        25.3535,
        27.3686,
        29.3698};
      vector<float> Y{
        0.918878,
        0.89659,
        0.869665,
        0.895808,
        0.94005,
        0.938376,
        0.897884,
        0.858437,
        0.825976,
        0.820597,
        0.823466,
        0.827613,
        0.829898,
        0.834169,
        0.836656,
        0.840727,
        0.840009};
      vector<float> X_low{
        5.29482,
        7.32926,
        9.30038,
        13.3208,
        15.3293,
        17.3446,
        21.3557,
        23.3466,
        29.3698};
      vector<float> Y_low{
        0.94005,
        0.938376,
        0.897884,
        0.825976,
        0.820597,
        0.823466,
        0.829898,
        0.834169,
        0.840009};
      vector<float> X_short{
        5.29482,
        6,
        7.32926,
        9.30038};
      vector<float> Y_short{
        0.94005,
        0.94,
        0.938376,
        0.897884};
      int n = 8;

      X = X_low;
      Y = Y_low;

      vector<float> coeff = polyfit(X, Y, n);
      vector<float> reg = polyval(X, coeff, n);

      plotManager->set_Xlabel("X");
      plotManager->set_Ylabel("Y");
      plotManager->set_Format_data1("with linespoints ls 1 pt 13 ps 0.5 lc rgb 'black' title 'Initial'");
      plotManager->set_Format_data2("with linespoints ls 1 pt 13 ps 0.5 lc rgb 'red' title 'Regression'");
      plotManager->plot_Regression(X, Y, reg);
      //-------------------

      coeff = polyfit_homemade(X, Y, n);
      reg = polyval(X, coeff, n);

      plotManager->set_Xlabel("X");
      plotManager->set_Ylabel("Y");
      plotManager->set_Format_data1("with linespoints ls 1 pt 13 ps 0.5 lc rgb 'black' title 'Initial'");
      plotManager->set_Format_data2("with linespoints ls 1 pt 13 ps 0.5 lc rgb 'red' title 'Regression'");
      plotManager->plot_Regression(X, Y, reg);
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_dataOpe = false;
    }
    ImGui::End();
  }
}

//Cloud infos
void GUI_windows::window_modifyFileInfo(){
  if( show_modifyFileInfo == true && sceneManager->is_atLeastOneMesh() == false){
    show_modifyFileInfo = false;
  }
  if(show_modifyFileInfo && sceneManager->is_atLeastOneMesh()){
    ImGui::Begin(ICON_FA_COMMENT " Point cloud", &show_modifyFileInfo, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoNav);
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    //Visibility
    static bool visible = true;
    if(ImGui::Checkbox("Visibility", &visible)){
      sceneManager->set_MeshVisibility(mesh, visible);
    }
    ImGui::SameLine();

    //Uniform cloud color
    static vec4 color_PC;
    ImGuiColorEditFlags flags = ImGuiColorEditFlags_NoInputs;
    flags |= ImGuiColorEditFlags_AlphaBar;
    if(ImGui::ColorEdit4("Color", (float*)&color_PC, flags)){
      if(sceneManager->is_atLeastOneMesh()){
        attribManager->set_pointCloudColor(mesh, color_PC);
      }
    }
    ImGui::Separator();

    //Name
    ImGui::Columns(2);
    static char str[256];
    strcpy(str, mesh->Name.c_str());
    ImGui::Text("Name ");
    ImGui::NextColumn();
    if(ImGui::InputText("##1", str, IM_ARRAYSIZE(str), ImGuiInputTextFlags_EnterReturnsTrue)){
      mesh->Name = str;
    }
    ImGui::NextColumn();

    //Format
    strcpy(str, mesh->Format.c_str());
    ImGui::Text("Format ");
    ImGui::NextColumn();
    if(ImGui::InputText("##2", str, IM_ARRAYSIZE(str), ImGuiInputTextFlags_EnterReturnsTrue)){
      mesh->Format = str;
    }
    ImGui::NextColumn();

    //Root pos
    vec3& PCroot = mesh->location.root;
    ImGui::Text("Root ");
    ImGui::NextColumn();
    ImGui::Text("%.2f, %.2f, %.2f", PCroot.x, PCroot.y, PCroot.z);
    ImGui::SameLine();
    if(ImGui::Button("R", ImVec2(15,0))){
      PCroot = vec3(0,0,0);
    }
    ImGui::NextColumn();

    //Attributs
    ImGui::TextWrapped("Attrib ");
    ImGui::NextColumn();
    ImGui::TextWrapped("%s", mesh->dataFormat.c_str());
    ImGui::Columns(1);
    ImGui::Separator();

    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Functions");
    if(ImGui::Button("Supress color", ImVec2(100,0))){
      mesh->color.hasData = !mesh->color.hasData;
      if(mesh->color.OBJ.size() == 0) mesh->color.hasData = false;
      sceneManager->update_dataFormat(mesh);
    }
    ImGui::SameLine();
    if(ImGui::Button("Supress normal", ImVec2(100,0))){
      mesh->normal.hasData = !mesh->normal.hasData;
      if(mesh->normal.OBJ.size() == 0) mesh->normal.hasData = false;
      sceneManager->update_dataFormat(mesh);
    }
    if(ImGui::Button("Transformation", ImVec2(100,0))){
      show_transformation = !show_transformation;
    }
    ImGui::SameLine();
    if(ImGui::Button("Data", ImVec2(100,0))){
      show_asciiData = true;
    }
    ImGui::Separator();

    //Statistics
    ImGui::TextColored(ImVec4(0.4f,0.4f,0.4f,1.0f),"Statistics");
    if(ImGui::Button("Location", ImVec2(100,0))){
      this->cloud_stats_location(mesh);
    }
    ImGui::SameLine();
    if(ImGui::Button("Intensity", ImVec2(100,0))){
      this->cloud_stats_intensity(mesh);
    }
    if(ImGui::Button("Distance", ImVec2(100,0))){
      this->cloud_stats_distance(mesh);
    }
    ImGui::SameLine();
    if(ImGui::Button("cos(It)", ImVec2(100,0))){
      this->cloud_stats_cosIt(mesh);
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Close")){
      show_modifyFileInfo = false;
    }
    ImGui::End();
  }
}
void GUI_windows::cloud_stats_location(Mesh* mesh){
  vector<vec3>& XYZ = mesh->location.OBJ;
  vec3 XYZ_COM = mesh->location.COM;
  vec3 XYZ_Min = mesh->location.Min;
  vec3 XYZ_Max = mesh->location.Max;
  //---------------------------

  vector<float> X, Y, Z;
  for(int i=0; i<XYZ.size(); i++){
    X.push_back(XYZ[i].x);
    Y.push_back(XYZ[i].y);
    Z.push_back(XYZ[i].z);
  }

  cout<<"---------"<<endl;
  cout<<"Name : "<<mesh->Name<<endl;

  if(XYZ.size() != 0){
    cout<<"___XYZ___"<<endl;
    cout<<"COM: "<<XYZ_COM.x<<" "<<XYZ_COM.y<<" "<<XYZ_COM.z<<endl;
    cout<<"Min: "<<XYZ_Min.x<<" "<<XYZ_Min.y<<" "<<XYZ_Min.z<<endl;
    cout<<"Max: "<<XYZ_Max.x<<" "<<XYZ_Max.y<<" "<<XYZ_Max.z<<endl;
    cout<<"Xaxis: min "<<Min(X)<<" <-> max "<<Max(X)<<" <-> mean "<<fct_Mean(X)<<endl;
    cout<<"Yaxis: min "<<Min(Y)<<" <-> max "<<Max(Y)<<" <-> mean "<<fct_Mean(Y)<<endl;
    cout<<"Zaxis: min "<<Min(Z)<<" <-> max "<<Max(Z)<<" <-> mean "<<fct_Mean(Z)<<endl;
    cout<<"__________"<<endl;
  }
}
void GUI_windows::cloud_stats_intensity(Mesh* mesh){
  vector<float>& Is = mesh->intensity.OBJ;
  //---------------------------

  cout<<"---------"<<endl;
  cout<<"Name : "<<mesh->Name<<endl;

  if(Is.size() != 0){
    cout<<"___Is___"<<endl;
    cout<<"Min : "<<Min(Is)<<endl;
    cout<<"Max : "<<Max(Is)<<endl;
    cout<<"Mean : "<<fct_Mean(Is)<<endl;
    cout<<"Var : "<<fct_var(Is)<<endl;
    cout<<"Std : "<<fct_std(Is)<<endl;
    cout<<"CV : "<<fct_CV(Is)<<endl;
    cout<<"__________"<<endl;
  }
}
void GUI_windows::cloud_stats_distance(Mesh* mesh){
  vector<float>& dist = mesh->attribut.dist;
  //---------------------------

  cout<<"---------"<<endl;
  cout<<"Name : "<<mesh->Name<<endl;

  //Distance
  if(dist.size() == 0){
    attribManager->compute_Distances(mesh);
  }
  cout<<"___Dist___"<<endl;
  cout<<"Min : "<<Min(dist)<<endl;
  cout<<"Max : "<<Max(dist)<<endl;
  cout<<"std : "<<fct_std(dist)<<endl;
  cout<<"Mean : "<<fct_Mean(dist)<<endl;
  cout<<"__________"<<endl;
}
void GUI_windows::cloud_stats_cosIt(Mesh* mesh){
  vector<float>& cosIt =  mesh->attribut.cosIt;
  vector<float>& It =  mesh->attribut.It;
  //---------------------------

  cout<<"---------"<<endl;
  cout<<"Name : "<<mesh->Name<<endl;

  //Angle d'incidence
  if(cosIt.size() == 0 || It.size() == 0){
    attribManager->compute_cosIt(mesh);
  }
  cout<<"___cosIt___"<<endl;
  cout<<"Min : "<<Min(cosIt)<<endl;
  cout<<"Max : "<<Max(cosIt)<<endl;
  cout<<"Mean : "<<fct_Mean(cosIt)<<endl;

  cout<<"___It___"<<endl;
  cout<<"Min : "<<Min(It)<<endl;
  cout<<"Max : "<<Max(It)<<endl;
  cout<<"std : "<<fct_std(It)<<endl;
  cout<<"Mean : "<<fct_Mean(It)<<endl;
  cout<<"__________"<<endl;
}
