#ifndef GUI_registration_H
#define GUI_registration_H

class Scene;
class Engine;
class ICP;
class Registration;
class Glyphs;
class Transforms;

#include "../Parameters.h"

class GUI_registration
{
public:
  //Constructor / Destructor
  GUI_registration(Engine* renderer);
  ~GUI_registration();

public:
  //Main function
  void design_Registration();

  //Subfunctions
  void regist_Color();
  void regist_DOF();
  void regist_Parameters();
  void regist_Registration();
  void regist_Stats();

private:
  Engine* engineManager;
  Scene* sceneManager;
  ICP* icpManager;
  Registration* regisManager;
  Glyphs* glyphManager;
  Transforms* transformManager;

  int regis_algo;
};

#endif
