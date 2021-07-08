#ifndef GUI_LEFTPANEL_H
#define GUI_LEFTPANEL_H

class GUI_radiometry;
class GUI_registration;
class GUI_matching;
class GUI_windows;

class Scene;
class Engine;
class Dimension;

#include "../Parameters.h"

class GUI_leftPanel
{
public:
  //Constructor / Destructor
  GUI_leftPanel(Engine* renderer, GUI_windows* winManager);
  ~GUI_leftPanel();

public:
  //Main function
  void design_leftPanel();
  void panel_top();
  void panel_bot();

  //Subfunctions
  void fileManager();
  void fileInfo(Mesh* mesh);
  void fileOptions(Mesh* mesh, bool cloudInfo);
  void workingModules();

  inline bool* get_module_correction(){return &module_correction;}
  inline bool* get_module_matching(){return &module_matching;}
  inline bool* get_module_registration(){return &module_registration;}
  inline float* get_panel_X(){return & panel_X;}

private:
  GUI_windows* gui_winManager;
  GUI_radiometry* gui_radioManager;
  GUI_matching* gui_matchManager;
  GUI_registration* gui_registManager;

  Scene* sceneManager;
  Engine* engineManager;
  Dimension* dimManager;

  ImVec2 XYmin, XYmax;
  float panel_Y;
  float panel_X;
  bool module_correction;
  bool module_matching;
  bool module_registration;
};

#endif
