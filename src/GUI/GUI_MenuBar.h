#ifndef GUI_MENUBAR_H
#define GUI_MENUBAR_H

#include "../Parameters.h"

class GUI_windows;
class GUI_option;
class GUI_leftPanel;

class Scene;
class Engine;
class Extraction;
class HeatMap;
class Textures;
class Operation;

class GUI_menuBar
{
public:
  //Constructor / Destructor
  GUI_menuBar(Engine* renderer, GUI_windows* winManager, GUI_option* options, GUI_leftPanel* leftPanel);
  ~GUI_menuBar();

public:
  //Main function
  void design_MenuBar();

  //Subfunctions
  void MenuBar_menus();
  void MenuBar_icons();
  void MenuBar_appInfo();
  void MenuBar_fastScene();
  void MenuBar_Operations();

private:
  GUI_windows* gui_winManager;
  GUI_option* optionManager;
  GUI_leftPanel* gui_leftPanelManager;

  Scene* sceneManager;
  Engine* engineManager;
  Operation* opeManager;
  Extraction* extractionManager;
  HeatMap* heatmapManager;
  Textures* texManager;
};

#endif
