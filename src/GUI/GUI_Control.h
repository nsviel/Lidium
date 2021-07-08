#ifndef GUI_control_H
#define GUI_control_H

#include "../Operation/Transforms.h"

#include "../Parameters.h"

class Scene;
class Engine;
class Camera;
class Registration;
class Attribut;
class Operation;
class Transforms;
class Extraction;
class Glyphs;
class Selection;
class Keypoint;
class HeatMap;
class Dimension;

class GUI_control
{
public:
  //Constructor / Destructor
  GUI_control(Engine* renderer);
  ~GUI_control();

public:
  //Main function
  void Gui_control();

  //Subfunctions
  void control_mouse();
  void control_frameSelection();
  void control_keyboard_oneAction();
  void control_keyboard_displace();
  void control_keyboard_ctrlAction();

  inline float* get_transCoef(){return &m_transCoef_slow;}
  inline float* get_rotatDegree(){return &rotatDegree;}

private:
  Transforms transformManager;

  Dimension* dimManager;
  Engine* engineManager;
  Camera* cameraManager;
  Registration* regisManager;
  Scene* sceneManager;
  Attribut* attribManager;
  Operation* opeManager;
  Extraction* extractionManager;
  Glyphs* glyphManager;
  Selection* selectionManager;
  Keypoint* keyManager;
  HeatMap* heatmapManager;

  float transCoef;
  float m_transCoef_slow, m_transCoef_fast;
  float rotatDegree;
};

#endif
