#ifndef DEFINITIONS_H
#define DEFINITIONS_H

//------Librairies------//
//STL
#include <algorithm>
#include <thread>
#include <string>
#include <vector>
#include <list>
#include <cstdio>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <time.h>
#include <utility>
#include <dirent.h>
#include <unistd.h>
#include <random>
#include <omp.h>
#include <experimental/filesystem>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/mman.h>

//OpenGL / OpenGLM
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//Others
#include <flann/flann.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>

//External
#include "../extern/imgui/imgui.h"
#include "../extern/imgui/imgui_impl_glfw.h"
#include "../extern/imgui/imgui_impl_opengl3.h"
#include "../extern/gnuplot/gnuplot-iostream.h"
#include "../extern/NormalHough/nanoflann.h"
#include "../extern/NormalHough/Normals.h"
#include "../extern/OptimLib/optim.hpp"
#include "../extern/nanoflann.hpp"
#include "../extern/IconsFontAwesome5.h"

//Internal
#include "Specific/struct_pointCloud.h"
#include "Specific/struct_glyph.h"
#include "Specific/struct_generic.h"
#include "Specific/fct_Transtypage.h"
#include "Specific/fct_Display.h"
#include "Specific/fct_OpenGL.h"
#include "Specific/fct_maths.h"
#include "Specific/Color.h"

//Include structures available for all
#include "Specific/struct_config.h"
extern struct Config configuration;
#include "Specific/struct_consol.h"
extern struct ConsoleApp console;

//------Namespaces------//
using namespace std;
using namespace glm;
using namespace Eigen;

#endif
