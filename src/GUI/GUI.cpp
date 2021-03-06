#include "GUI.h"
#include "GUI_windows.h"
#include "GUI_Control.h"
#include "GUI_Option.h"
#include "GUI_MenuBar.h"
#include "GUI_LeftPanel.h"
#include "GUI_Consol.h"

#include "../Engine/Engine.h"
#include "../Engine/OpenGL/Camera.h"
#include "../Engine/Dimension.h"
#include "../Specific/struct_consol.h"

//Consol creation
ConsoleApp console;

//Constructor / Destructor
GUI::GUI(Engine* engine){
  this->engineManager = engine;
  //---------------------------

  this->dimManager = engineManager->get_dimManager();
  this->gui_winManager = new GUI_windows(engineManager);
  this->gui_controlManager = new GUI_control(engineManager);
  this->gui_optionManager = new GUI_option(engineManager, gui_controlManager);
  this->gui_leftPanelManager = new GUI_leftPanel(engineManager, gui_winManager);
  this->gui_menuBarManager = new GUI_menuBar(engineManager, gui_winManager, gui_optionManager, gui_leftPanelManager);
  this->gui_consol = new GUI_consol(engineManager);

  //---------------------------
  this->Gui_init();
  this->Gui_style();
  this->Gui_font();
}
GUI::~GUI(){}

//Main functions
void GUI::Gui_init(){
  //---------------------------

  //ImGui stuff
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  GLFWwindow* window = dimManager->get_window();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  //---------------------------
}
void GUI::Gui_style(){
  ImGuiStyle& style = ImGui::GetStyle();
  ImGuiIO io = ImGui::GetIO();
  //---------------------------

  //Formatting
  style.FrameBorderSize = 0.0f;
  style.WindowPadding.x = 5.0f;

  //Rounding
  style.WindowRounding = 0.0f;
  style.TabRounding = 0.0f;
  style.GrabRounding = 0.0f;
  style.ScrollbarRounding = 0.0f;
  style.ChildRounding = 0.0f;
  style.FrameRounding = 0.0f;
  style.PopupRounding = 0.0f;
  style.IndentSpacing = 0.0f; //Pas d'indentation des tree

  //Colors
  ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(112, 112, 112, 127)); //Frame background
  ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(200, 200, 200, 127)); //Button color
  ImGui::PushStyleColor(ImGuiCol_Header, IM_COL32(76, 76, 76, 124));
  ImGui::PushStyleColor(ImGuiCol_Tab, IM_COL32(117, 117, 117, 220));

  //IO parameters
  io.ConfigWindowsResizeFromEdges = true;

  //---------------------------
}
void GUI::Gui_font(){
  ImGuiIO io = ImGui::GetIO();
  static const ImWchar icons_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
  ImFontConfig config;
  config.MergeMode = true;
  config.GlyphMinAdvanceX = 15.0f; //Monospace icons
  //---------------------------

  //Load font size 12
  io.Fonts->AddFontFromFileTTF("../media/engine/font/DroidSans.ttf", 12.0f);
  io.Fonts->AddFontFromFileTTF("../media/engine/font/fontawesome-webfont.ttf", 12.5f, &config, icons_ranges);

  //Load font size 13
  io.Fonts->AddFontFromFileTTF("../media/engine/font/DroidSans.ttf", 13.0f);
  io.Fonts->AddFontFromFileTTF("../media/engine/font/fontawesome-webfont.ttf", 12.5f, &config, icons_ranges);

  //Buid the font database
  io.Fonts->Build();

  //---------------------------
}
void GUI::Gui_loop(){
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  //---------------------------

  gui_consol->design_consol();
  gui_controlManager->Gui_control();
  gui_menuBarManager->design_MenuBar();
  gui_leftPanelManager->design_leftPanel();
  gui_winManager->window_Draw();

  this->Gui_Dimensions();

  //---------------------------
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

//Subfunctions
void GUI::Gui_Dimensions(){
  float* leftPanel_X = gui_leftPanelManager->get_panel_X();
  float* bottomPanel_X = gui_consol->get_panel_X();
  float* bottomPanel_Y = gui_consol->get_panel_Y();
  //---------------------------

  gui_consol->set_panel_X(*leftPanel_X);
  dimManager->set_guiDim_lP(vec2(*leftPanel_X, 0));
  dimManager->set_guiDim_bP(vec2(0, *bottomPanel_Y));

  //---------------------------
}
void GUI::Gui_bkgColor(vec3* value){
  gui_optionManager->set_backgroundColorPtr(value);
}
