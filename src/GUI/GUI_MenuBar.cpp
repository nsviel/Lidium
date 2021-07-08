#include "GUI_MenuBar.h"
#include "GUI_windows.h"
#include "GUI_Option.h"
#include "GUI_LeftPanel.h"

#include "../Engine/Scene.h"
#include "../Engine/Engine.h"
#include "../Operation/Operation.h"
#include "../Operation/Functions/Extraction.h"
#include "../Operation/Functions/Heatmap.h"
#include "../Operation/Transforms.h"
#include "../Engine/OpenGL/Textures.h"

//Constructor / Destructor
GUI_menuBar::GUI_menuBar(Engine* renderer, GUI_windows* winManager, GUI_option* options, GUI_leftPanel* leftPanel){
  this->optionManager = options;
  this->gui_winManager = winManager;
  this->engineManager = renderer;
  this->gui_leftPanelManager = leftPanel;
  //---------------------------

  this->sceneManager = engineManager->get_SceneManager();
  this->heatmapManager = engineManager->get_heatmapManager();
  this->texManager = engineManager->get_texManager();
  this->extractionManager = engineManager->get_extractionManager();
  this->opeManager = engineManager->get_OpeManager();

  //---------------------------
}
GUI_menuBar::~GUI_menuBar(){}

//Main function
void GUI_menuBar::design_MenuBar(){
  if(ImGui::BeginMainMenuBar()){
    //------------------------

    this->MenuBar_menus();
    this->MenuBar_icons();
    this->MenuBar_appInfo();

    //-------------------------
    ImGui::EndMainMenuBar();
  }
}

//Subfunctions
void GUI_menuBar::MenuBar_menus(){
  //-------------------------

  if (ImGui::BeginMenu("File")){
    if (ImGui::MenuItem("Open","ctrl+w")){
      opeManager->loading();
    }
    if (ImGui::MenuItem("Open parameters")){
      *gui_winManager->get_show_openOptions() = true;
    }

    if (ImGui::MenuItem("Save as","ctrl+s")){
      opeManager->saving();
    }
    if (ImGui::MenuItem("Save options")){
      *gui_winManager->get_show_saveOptions() = true;
    }
    if (ImGui::MenuItem("Save all")){
      opeManager->allSaving();
    }
    if(ImGui::MenuItem("Remove", "Suppr")){
      sceneManager->removeCloud(sceneManager->get_selectedMesh());
    }
    if(ImGui::MenuItem("Remove all")){
      sceneManager->removeCloud_all();
    }
    if(ImGui::MenuItem("Next","tab")){
      sceneManager->select_nextMesh();
    }
    if(ImGui::MenuItem("Center","c")){
      if(!sceneManager->is_listMeshEmpty()){
        Mesh* mesh = sceneManager->get_selectedMesh();
        Transforms transformManager;
        transformManager.make_centering(mesh);
        sceneManager->update_CloudPosition(mesh);
      }
    }
    if(ImGui::MenuItem("Reset","r")){
      opeManager->reset();
    }

    ImGui::EndMenu();
  }
  if (ImGui::MenuItem(ICON_FA_FILE " Open")){
    opeManager->loading();
  }
  if (ImGui::BeginMenu(ICON_FA_COG " Option")){
    optionManager->design_Options();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Operation")){
    this->MenuBar_Operations();
    ImGui::EndMenu();
  }
  if(ImGui::BeginMenu("Modules")){
    //Radiometric correction
    bool* module_corr =  gui_leftPanelManager->get_module_correction();
    ImGui::Checkbox("Radio correction", module_corr);

    //Radiometric correction
    bool* module_match =  gui_leftPanelManager->get_module_matching();
    ImGui::Checkbox("Matching", module_match);

    //Radiometric correction
    bool* module_regis =  gui_leftPanelManager->get_module_registration();
    ImGui::Checkbox("Registration", module_regis);

    ImGui::EndMenu();
  }
  if(ImGui::BeginMenu("Init")){
    this->MenuBar_fastScene();
    ImGui::EndMenu();
  }

  //-------------------------
}
void GUI_menuBar::MenuBar_icons(){
  ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(22, 110, 161, 255));
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 2.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2, 0));
  float iconSize = 0;
  Texture* texture;
  //---------------------------

  //Cloud
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 150);
  if(ImGui::Button(ICON_FA_COMMENT, ImVec2(iconSize,iconSize))){
    if(sceneManager->is_atLeastOneMesh()){
      bool* ptr = gui_winManager->get_show_modifyFileInfo();
      *ptr = !*ptr;
    }
  }
  if(ImGui::IsItemHovered()){
    ImGui::SetTooltip("Cloud info");
  }

  //Heatmap
  //ImGui::SetCursorPosX(ImGui::GetCursorPosX() - 10);
  if(ImGui::Button(ICON_FA_EYE, ImVec2(iconSize,iconSize))){
    if(sceneManager->is_atLeastOneMesh()){
      //Apply heatmap
      Mesh* mesh = sceneManager->get_selectedMesh();
      heatmapManager->set_normalized(false);
      heatmapManager->set_HeatMap(mesh);

      //Heatmap window
      bool* ptr = gui_winManager->get_show_heatmap();
      *ptr = !*ptr;
    }
  }
  if(ImGui::IsItemHovered()){
    ImGui::SetTooltip("Heatmap");
  }

  //Camera
  //ImGui::SetCursorPosX(ImGui::GetCursorPosX() - 2);
  if(ImGui::Button(ICON_FA_CAMERA, ImVec2(iconSize,iconSize))){
    bool* ptr = gui_winManager->get_show_camera();
    *ptr = !*ptr;
  }
  if(ImGui::IsItemHovered()){
    ImGui::SetTooltip("Camera");
  }

  //---------------------------
  ImGui::PopStyleVar(2);
  ImGui::PopStyleColor(1);
}
void GUI_menuBar::MenuBar_fastScene(){
  //---------------------------

  //Simple clouds
  if(ImGui::Button("Buddha", ImVec2(100,0))){
    opeManager->fastScene(0);
  }
  ImGui::SameLine();
  if(ImGui::Button("Torus", ImVec2(100,0))){
    opeManager->fastScene(1);
  }

  //AB clouds
  if(ImGui::Button("AB sparse", ImVec2(100,0))){
    opeManager->fastScene(2);
  }
  ImGui::SameLine();
  if(ImGui::Button("AB dense", ImVec2(100,0))){
    opeManager->fastScene(3);
  }

  //Sphere calibration targets
  if(ImGui::Button("Spheres S3-80°", ImVec2(100,0))){
    opeManager->fastScene(4);
  }
  ImGui::SameLine();
  if(ImGui::Button("Spheres+add-80°", ImVec2(100,0))){
    opeManager->fastScene(5);
  }

  //Spectralon calibration targets
  if(ImGui::Button("Spect. distance", ImVec2(100,0))){
    opeManager->fastScene(6);
  }
  ImGui::SameLine();
  if(ImGui::Button("Spect. angle", ImVec2(100,0))){
    opeManager->fastScene(7);
  }
  if(ImGui::Button("Spect. complete", ImVec2(100,0))){
    opeManager->fastScene(8);
  }
  ImGui::SameLine();
  if(ImGui::Button("Spectralon 5m", ImVec2(100,0))){
    opeManager->fastScene(9);
  }

  //Test clouds for keypoint matching
  if(ImGui::Button("Keyp AB_12", ImVec2(100,0))){
    opeManager->fastScene(13);
  }
  ImGui::SameLine();
  if(ImGui::Button("Keyp AB_11", ImVec2(100,0))){
    opeManager->fastScene(14);
  }
  if(ImGui::Button("Chapelle 01-03", ImVec2(100,0))){
    opeManager->fastScene(16);
  }
  ImGui::SameLine();
  if(ImGui::Button("Tableau 01-03", ImVec2(100,0))){
    opeManager->fastScene(17);
  }

  //---------------------------
}
void GUI_menuBar::MenuBar_Operations(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  //---------------------------

  if(ImGui::Button("File treatment", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_fileTreatment();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Extract cloud", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_extractCloud();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Cut cloud", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_cutCloud();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Selection", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_selection();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Filter", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_filtering();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Merge clouds", ImVec2(150,0))){
    if(sceneManager->get_listMeshSize() >= 2){
      Mesh* mesh_2 = sceneManager->get_otherMesh();
      extractionManager->fct_merging_newCloud(mesh, mesh_2);
    }
  }
  if(ImGui::Button("Normal", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_normal();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Intensity", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_intensity();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Color", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_color();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Data", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_dataOpe();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Fitting", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_fitting();
    *bool_ptr = !*bool_ptr;
  }
  if(ImGui::Button("Transformation", ImVec2(150,0))){
    bool* bool_ptr = gui_winManager->get_show_transformation();
    *bool_ptr = !*bool_ptr;
  }

  //---------------------------
}
void GUI_menuBar::MenuBar_appInfo(){
  ImGui::SameLine(ImGui::GetWindowWidth()-50);
  if(ImGui::BeginMenu("Infos")){
    //---------------------------

    //OpenGl version
    const char* oglv = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    ImGui::MenuItem("OGL v.", oglv);
    ImGui::Separator();

    //RAM memory
    const double megabyte = 1024 * 1024;
    struct sysinfo si;
    sysinfo (&si);
    float percentFreeRam = ((float)si.freeram*100) / (float)si.totalram;
    ImGui::Text("total RAM   : %5.1f MB\n", si.totalram / megabyte);
    ImGui::Text("free RAM   : %5.1f MB - %.1f%%\n", si.freeram / megabyte, percentFreeRam);
    ImGui::Separator();

    //Framerate
    ImGuiIO io = ImGui::GetIO();
    ImGui::Text("%.1f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);

    //---------------------------
    ImGui::EndMenu();
  }
}
