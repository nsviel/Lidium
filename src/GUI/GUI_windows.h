#ifndef GUI_WINDOWS_H
#define GUI_WINDOWS_H

#include "../Load/Loader.h"
#include "../Operation/Transforms.h"

#include "../Parameters.h"

class Engine;
class Camera;
class Fusion;
class Filter;
class Scene;
class Attribut;
class Transforms;
class Radiometry;
class Operation;
class Extraction;
class Glyphs;
class HeatMap;
class Selection;
class Loader;
class Plotting;
class Fitting;

class GUI_windows
{
public:
  //Constructor / Destructor
  GUI_windows(Engine* renderer);
  ~GUI_windows();

public:
  //Main functions
  void init();
  void window_Draw();

  //General windows
  void window_asciiData();
  void window_loadOption();
  void window_camera();
  void window_saveOption();
  void window_transformation();
  void window_selection();
  void window_extractCloud();
  void window_cutCloud();
  void window_filter();
  void window_fitting();
  void window_normal();
  void window_intensity();
  void window_color();
  void window_fileTreatment();
  void window_dataOpe();
  void window_heatmap();

  //Cloud infos
  void window_modifyFileInfo();
  void cloud_stats_location(Mesh* mesh);
  void cloud_stats_intensity(Mesh* mesh);
  void cloud_stats_distance(Mesh* mesh);
  void cloud_stats_cosIt(Mesh* mesh);

  inline bool* get_show_asciiData(){return &show_asciiData;}
  inline bool* get_show_selection(){return &show_selection;}
  inline bool* get_show_openOptions(){return &show_openOptions;}
  inline bool* get_show_camera(){return &show_camera;}
  inline bool* get_show_heatmap(){return &show_heatmap;}
  inline bool* get_show_transformation(){return &show_transformation;}
  inline bool* get_show_filtering(){return &show_filtering;}
  inline bool* get_show_fitting(){return &show_fitting;}
  inline bool* get_show_saveOptions(){return &show_saveOptions;}
  inline bool* get_show_modifyFileInfo(){return &show_modifyFileInfo;}
  inline bool* get_show_extractCloud(){return &show_extractCloud;}
  inline bool* get_show_cutCloud(){return &show_cutCloud;}
  inline bool* get_show_normal(){return &show_normal;}
  inline bool* get_show_intensity(){return &show_intensity;}
  inline bool* get_show_color(){return &show_color;}
  inline bool* get_show_fileTreatment(){return &show_fileTreatment;}
  inline bool* get_show_dataOpe(){return &show_dataOpe;}
  inline bool* get_cloud_movement(){return &cloud_movement;}

private:
  Loader loaderManager;
  Engine* engineManager;
  Camera* cameraManager;
  Fusion* fusionManager;
  Filter* filterManager;
  Scene* sceneManager;
  Attribut* attribManager;
  Radiometry* radioManager;
  Glyphs* glyphManager;
  HeatMap* heatmapManager;
  Selection* selectionManager;
  Plotting* plotManager;
  Fitting* fitManager;
  Operation* opeManager;
  Transforms transformManager;

  bool show_asciiData;
  bool show_selection;
  bool show_openOptions;
  bool show_camera;
  bool show_transformation;
  bool show_filtering;
  bool show_fitting;
  bool show_saveOptions;
  bool show_modifyFileInfo;
  bool show_heatmap;
  bool show_extractCloud;
  bool show_cutCloud;
  bool show_normal;
  bool show_intensity;
  bool show_color;
  bool show_fileTreatment;
  bool show_dataOpe;
  bool cloud_movement;
};

#endif
