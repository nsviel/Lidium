#ifndef GUI_H
#define GUI_H

class GUI_windows;
class GUI_control;
class GUI_option;
class GUI_menuBar;
class GUI_leftPanel;
class GUI_consol;

class Engine;
class Camera;
class Dimension;

#include "../Parameters.h"

class GUI
{
public:
  //Constructor / Destructor
  GUI(Engine* engine);
  ~GUI();

public:
  //Main functions
  void Gui_init();
  void Gui_style();
  void Gui_font();
  void Gui_loop();

  //Subfunctions
  void Gui_Dimensions();
  void Gui_bkgColor(vec3* value);

private:
  GUI_windows* gui_winManager;
  GUI_control* gui_controlManager;
  GUI_option* gui_optionManager;
  GUI_menuBar* gui_menuBarManager;
  GUI_leftPanel* gui_leftPanelManager;
  GUI_consol* gui_consol;

  Engine* engineManager;
  Camera* cameraManager;
  Dimension* dimManager;
};

#endif
