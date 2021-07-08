#include "GUI_LeftPanel.h"
#include "GUI_Radiometry.h"
#include "GUI_Registration.h"
#include "GUI_Matching.h"
#include "GUI_windows.h"

#include "../Engine/Scene.h"
#include "../Engine/Engine.h"
#include "../Engine/Dimension.h"

//Constructor / Destructor
GUI_leftPanel::GUI_leftPanel(Engine* renderer, GUI_windows* winManager){
  this->gui_winManager = winManager;
  this->engineManager = renderer;
  //-------------------------------

  this->sceneManager = engineManager->get_SceneManager();
  this->dimManager = engineManager->get_dimManager();

  this->gui_radioManager = new GUI_radiometry(engineManager, gui_winManager);
  this->gui_matchManager = new GUI_matching(engineManager, gui_winManager);
  this->gui_registManager = new GUI_registration(engineManager);

  this->module_correction = true;
  this->module_matching = true;
  this->module_registration = true;

  this->panel_X = configuration.GUI_LeftPanel_width;
  this->panel_Y = configuration.GUI_LeftPanel_mid;

  //-------------------------------
}
GUI_leftPanel::~GUI_leftPanel(){}

//Main function
void GUI_leftPanel::design_leftPanel(){
  //----------------------------

  this->panel_top();
  this->panel_bot();

  //----------------------------
}
void GUI_leftPanel::panel_top(){
  vec2 guiDim_tP = dimManager->get_guiDim_tP();
  vec2 winDim = dimManager->get_winDim();
  ImVec2 lp_min = ImVec2(10, 10);
  ImVec2 lp_max = ImVec2(500, winDim.y - guiDim_tP.y);
  //----------------------------

  //Options
  ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBringToFrontOnFocus;
  ImGui::SetNextWindowPos(ImVec2(0, guiDim_tP.y));
  ImGui::SetNextWindowSize(ImVec2(panel_X, panel_Y - guiDim_tP.y));
  ImGui::SetNextWindowSizeConstraints(lp_min, lp_max);
  ImGui::Begin("LeftPanel##topOuter", NULL, window_flags);{

    //Update panel dimension
    panel_X = ImGui::GetWindowSize().x;
    panel_Y = ImGui::GetWindowSize().y + guiDim_tP.y;

    //Set inner window
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
    ImGui::SetNextWindowPos(ImVec2(0, guiDim_tP.y));
    ImGui::SetNextWindowSize(ImVec2(panel_X - 1, panel_Y - guiDim_tP.y - 1));
    ImGui::Begin("LeftPanel##topInner", NULL, window_flags);{

      //File management
      this->fileManager();

    }
    ImGui::PopStyleVar();
  }

  ImGui::End();

  //----------------------------
}
void GUI_leftPanel::panel_bot(){
  vec2 guiDim_tP = dimManager->get_guiDim_tP();
  vec2 winDim = dimManager->get_winDim();
  ImVec2 lp_min = ImVec2(10, 10);
  ImVec2 lp_max = ImVec2(500, winDim.y - guiDim_tP.y);
  //----------------------------

  //Options
  ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBringToFrontOnFocus;
  ImGui::SetNextWindowPos(ImVec2(0, panel_Y));
  ImGui::SetNextWindowSize(ImVec2(panel_X, winDim.y - panel_Y));
  ImGui::SetNextWindowSizeConstraints(lp_min, lp_max);
  ImGui::Begin("LeftPanel##botOuter", NULL, window_flags);{

    //Update panel dimension
    panel_X = ImGui::GetWindowSize().x;
    panel_Y = winDim.y - ImGui::GetWindowSize().y;

    //Set inner window
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar;
    ImGui::SetNextWindowPos(ImVec2(0, panel_Y + 1));
    ImGui::SetNextWindowSize(ImVec2(panel_X - 1, winDim.y - panel_Y - 1));
    ImGui::Begin("LeftPanel##botInner", NULL, window_flags);{

      //Working areas
      this->workingModules();

    }
    ImGui::PopStyleVar();
  }

  ImGui::End();

  //----------------------------
}

//Subfunctions
void GUI_leftPanel::fileManager(){
  vec2 guiDim_tP = dimManager->get_guiDim_tP();
  vec2 winDim = dimManager->get_winDim();
  //-------------------------------

  //Model tab
  ImGuiWindowFlags window_flags = ImGuiWindowFlags_AlwaysVerticalScrollbar;
  ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 0, 0, 255));
  ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(255, 255, 255, 255));
  ImGui::BeginChild("##ScrollingRegion", ImVec2(ImGui::GetWindowContentRegionWidth(), panel_Y - 40), true);

  //Get list of all loaded files
  list<Mesh*>* list_mesh = sceneManager->get_listMesh();
  int selected = sceneManager->get_orderSelectedMesh();
  static int selection_mask = (1 << 2);

  //Display loaded files
  for(int i=0; i<list_mesh->size(); i++){
    Mesh* mesh = *next(list_mesh->begin(),i);

    //File selection
    ImGuiTreeNodeFlags node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
    if(selection_mask & (1 << i)){
      node_flags |= ImGuiTreeNodeFlags_Selected;
    }
    bool cloudInfo = false;
    if(ImGui::TreeNodeEx((void*)(intptr_t)i, node_flags, "%s", mesh->Name.c_str())){
      cloudInfo = true;
      ImGui::TreePop();
    }
    if(ImGui::IsItemClicked()){
      sceneManager->set_selectedMesh(mesh);
      selected = i;
    }
    if(selected != -1){
      selection_mask = (1 << selected);
    }

    //File options
    this->fileOptions(mesh, cloudInfo);
  }

  //-------------------------------
  ImGui::EndChild();
  ImGui::PopStyleColor();
  ImGui::PopStyleColor();
  ImGui::Columns(1);
  ImGui::Separator();
  ImGui::TreePop();
}
void GUI_leftPanel::fileInfo(Mesh* mesh){
  vec3& COM = mesh->location.COM;
  vec3& PCroot = mesh->location.root;
  vector<vec3>& XYZ = mesh->location.OBJ;
  ImGui::Separator();
  //---------------------------

  sceneManager->update_dataFormat(mesh);

  //Additional info
  static char str[256];
  strcpy(str, mesh->Format.c_str());
  ImGui::TextWrapped("Data: %s", mesh->dataFormat.c_str());
  ImGui::Text("Format: %s", str);
  ImGui::Text("Pts: %lu", XYZ.size());
  ImGui::Text("COM (%.2f, %.2f, %.2f)", COM.x, COM.y, COM.z);
  ImGui::Text("Z [%.2f; %.2f]", mesh->location.Min.z, mesh->location.Max.z);

  //---------------------------
  ImGui::Separator();
}
void GUI_leftPanel::fileOptions(Mesh* mesh, bool cloudInfo){
  //---------------------------

  //Removal cross
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f,1.0f,1.0f,1.0f));
  ImGui::PushID(mesh->oID);
  ImGui::SameLine(ImGui::GetWindowWidth()-25);
  if(ImGui::SmallButton(ICON_FA_TRASH)){
    sceneManager->removeCloud(mesh);
  }

  //Modification window
  ImGui::SameLine(ImGui::GetWindowWidth()-50);
  if(ImGui::SmallButton(ICON_FA_CLIPBOARD)){
    bool* ptr = gui_winManager->get_show_modifyFileInfo();
    *ptr = !*ptr;
  }
  ImGui::PopID();
  ImGui::PopStyleColor();

  //---------------------------
  if(cloudInfo){
    this->fileInfo(mesh);
  }
}
void GUI_leftPanel::workingModules(){
  if(ImGui::BeginTabBar("##tabs", ImGuiTabBarFlags_None)){
    //-------------------------------

    if(module_correction)
    if(ImGui::BeginTabItem("Radiometry")){
      gui_radioManager->design_Radiometry();
      ImGui::EndTabItem();
    }

    if(module_matching)
    if(ImGui::BeginTabItem("Matching")){
      gui_matchManager->design_Matching();
      ImGui::EndTabItem();
    }

    if(module_registration)
    if(ImGui::BeginTabItem("Registration")){
      gui_registManager->design_Registration();
      ImGui::EndTabItem();
    }

    //-------------------------------
    ImGui::EndTabBar();
  }
}
