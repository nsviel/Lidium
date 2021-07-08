#ifndef OPERATION_H
#define OPERATION_H

class Scene;
class Glyphs;

#include "../Parameters.h"

class Operation
{
public:
  //Constructor / Destructor
  Operation(Scene* scene, Glyphs* glyph);
  ~Operation();

public:
  void reset();
  void fastScene(int mode);
  void loading();
  void loading(string folderPath);
  void selectDirectory(string* folderPath);
  void loading_sampling();
  void loading_treatment();
  void loadingFolder(string path);
  void samplingLoader(string path);
  void saving();
  void allSaving();
  void convertIscale();
  void detectSphere();

  inline void set_spaceSampling(float value){this->spaceSampling = value;}
  inline void set_nbLineSampling(int value){this->nbLineSampling = value;}

private:
  Scene* sceneManager;
  Glyphs* glyphManager;

  float spaceSampling;
  int nbLineSampling;
  string pathDir;
  uint modelID, comID;
};

#endif
