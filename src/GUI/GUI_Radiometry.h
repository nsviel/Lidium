#ifndef GUI_radiometry_H
#define GUI_radiometry_H

class GUI_windows;

class Engine;
class Scene;
class Operation;
class Radiometry;
class HeatMap;
class Reference;
class Linearization;

#include "../Parameters.h"

class GUI_radiometry
{
public:
  //Constructor / Destructor
  GUI_radiometry(Engine* renderer, GUI_windows* winManager);
  ~GUI_radiometry();

public:
  //Main function
  void design_Radiometry();

  //Subfunctions
  void cloudStat();
  void plotting();
  void calibrationTargets();
  void options();
  void Approaches();
  void correction();

private:
  GUI_windows* gui_winManager;

  Linearization* linManager;
  Engine* engineManager;
  Scene* sceneManager;
  Radiometry* radioManager;
  HeatMap* heatmapManager;
  Reference* refManager;
  Operation* opeManager;

  int corr_num;
  int corr_data;
  bool corr_heat;
  bool corr_ref;
};

#endif
