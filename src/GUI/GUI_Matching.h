#ifndef GUI_MATCHING_H
#define GUI_MATCHING_H

class GUI_windows;

class Scene;
class Engine;
class Glyphs;
class Attribut;
class ICP;
class Keypoint;
class RegionGrowing;
class Radiometry;
class Linearization;

#include "../Parameters.h"

class GUI_matching
{
public:
  GUI_matching(Engine* renderer, GUI_windows* winManager);
  ~GUI_matching();

public:
  //Main function
  void design_Matching();

  //Subcategories
  void match_matching();
  void match_keypoint();
  void match_dense();
  void match_methods();

  //Subfunctions
  void keypoint_options();
  void keypoint_parameters();
  void keypoint_rejection();

private:
  GUI_windows* gui_winManager;

  Engine* engineManager;
  Scene* sceneManager;
  Glyphs* glyphManager;
  Attribut* attribManager;
  ICP* icpManager;
  Keypoint* keyManager;
  RegionGrowing* growingManager;
  Radiometry* radioManager;
};

#endif
