#include "Engine/OpenGL/CoreGLengine.h"

/**
 * \file main.cpp
 * \brief Start the software
 * \author Nathan Sanchiz-Viel
 */

int main(int argc, char *argv[])
{
  cout<<"--Begin program--"<<endl;
  //---------------------------

  CoreGLengine GLengine;
  GLengine.init();
  GLengine.loop();

  //---------------------------
  cout<<"--Close program--"<<endl;
  return 0;
}
