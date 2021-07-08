#include "GUI_Control.h"

#include "../Engine/Glyphs.h"
#include "../Engine/Scene.h"
#include "../Engine/Engine.h"
#include "../Operation/Attribut.h"
#include "../Operation/Operation.h"
#include "../Engine/OpenGL/Camera.h"
#include "../Engine/Dimension.h"
#include "../Operation/Functions/Selection.h"
#include "../Operation/Functions/Extraction.h"
#include "../Operation/Functions/Heatmap.h"
#include "../Operation/Functions/Segmentation.h"
#include "../Registration/Registration.h"
#include "../Registration/Pipeline/ICP.h"
#include "../Registration/Pipeline/Keypoint.h"

//Constructor / Destructor
GUI_control::GUI_control(Engine* engine){
  this->engineManager = engine;
  //---------------------------

  this->cameraManager = engineManager->get_CameraManager();
  this->dimManager = engineManager->get_dimManager();
  this->sceneManager = engineManager->get_SceneManager();
  this->attribManager = engineManager->get_AttribManager();
  this->opeManager = engineManager->get_OpeManager();
  this->glyphManager = engineManager->get_glyphManager();
  this->extractionManager = engineManager->get_extractionManager();
  this->heatmapManager = engineManager->get_heatmapManager();
  this->selectionManager = engineManager->get_selectionManager();
  this->regisManager = engineManager->get_regisManager();
  ICP* icpManager = regisManager->get_icpManager();
  this->keyManager = icpManager->get_keyManager();

  this->m_transCoef_slow = configuration.TRANSFORM_Trans;
  this->m_transCoef_fast = configuration.TRANSFORM_TransFast;
  this->rotatDegree = configuration.TRANSFORM_Rot;

  //---------------------------
}
GUI_control::~GUI_control(){}

//Main function
void GUI_control::Gui_control(){
  //---------------------------

  this->control_mouse();
  this->control_frameSelection();
  this->control_keyboard_oneAction();
  this->control_keyboard_displace();
  this->control_keyboard_ctrlAction();

  //---------------------------
}

//Subfunctions
void GUI_control::control_mouse(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  ImGuiIO io = ImGui::GetIO();
  GLFWwindow* window = dimManager->get_window();
  //----------------------------

  static int PCrotMode = 2;
  //Wheel - Mesh rotation
  if(io.MouseWheel && io.MouseDownDuration[1] == -1 && !io.WantCaptureMouse){
    if(!sceneManager->is_listMeshEmpty()){
      float rotation = io.MouseWheel * rotatDegree*M_PI/180;
      vec3 COM = mesh->location.COM;

      if(PCrotMode == 0) transformManager.make_rotation(mesh, COM, vec3(rotation,0,0));
      if(PCrotMode == 1) transformManager.make_rotation(mesh, COM, vec3(0,rotation,0));
      if(PCrotMode == 2) transformManager.make_rotation(mesh, COM, vec3(0,0,rotation));
      sceneManager->update_CloudPosition(mesh);
    }
  }

  //Wheel - Camera zoom
  if(io.MouseWheel && io.MouseDownDuration[1] >= 0.0f && !io.WantCaptureMouse){
    cameraManager->compute_positionalZoom(io.MouseWheel);
  }

  //Middle click - Change mesh rotation axis
  if(ImGui::IsMouseClicked(2) && !io.WantCaptureMouse){
    PCrotMode++;
    if(PCrotMode >= 3) PCrotMode = 0;
  }

  //Right click - Camera movement
  static vec2 cursorPos;
  if(ImGui::IsMouseClicked(1) && !io.WantCaptureMouse){

    //Save cursor position
    cursorPos = dimManager->get_cursorPos();

    //Hide cursor
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

    //Set cursor to screen middle
    vec2 glMiddle = dimManager->get_glMiddle();
    glfwSetCursorPos(window, glMiddle.x, glMiddle.y);

    //Enable camera movement
    cameraManager->set_cameraMovON(true);

  }
  if(ImGui::IsMouseReleased(1)){//} && !io.WantCaptureMouse){
    //Restaure cursor position
    dimManager->set_cursorPos(cursorPos);

    //Disable camera movement
    cameraManager->set_cameraMovON(false);
  }
  if(io.MouseDown[1] && !io.WantCaptureMouse){
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
  }

  //Left click - Point selection
  if(ImGui::IsMouseClicked(0) && !io.WantCaptureMouse){
    //vec3 point = selectionManager->mouse_clickedPoint();
    //selectionManager->selectionPoint(point);
  }

  //---------------------------
}
void GUI_control::control_frameSelection(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  ImGuiIO io = ImGui::GetIO();
  //----------------------------

  //Frame selection
  static vec2 pt1, pt2;
  if(ImGui::IsKeyPressed(341) == false){
    if(ImGui::IsMouseClicked(0) && io.MouseDownDuration[0] < 0.01f && !io.WantCaptureMouse){
      pt1 = dimManager->get_cursorPos_gl();
    }
    if(io.MouseDownDuration[0] >= 0.01f && !io.WantCaptureMouse){
      pt2 = dimManager->get_cursorPos_gl();
      selectionManager->mouse_drawFrame(pt1, pt2);
    }
    if(ImGui::IsMouseReleased(0) && !io.WantCaptureMouse){
      selectionManager->mouse_frameSelection(pt1, pt2);
    }
  }

  //----------------------------
}
void GUI_control::control_keyboard_oneAction(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  ImGuiIO io = ImGui::GetIO();
  GLFWwindow* window = dimManager->get_window();
  //----------------------------

  for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++){
    //Esc key
    if(ImGui::IsKeyPressed(256)){
      glfwSetWindowShouldClose(window, true);
    }

    //I key - ICP
    if (ImGui::IsKeyPressed(73) && !io.WantCaptureMouse){
      if(sceneManager->is_atLeastMinNbMesh(2)){
        regisManager->make_Iteration();
      }
      break;
    }

    //Tab key
    if (ImGui::IsKeyPressed(258)){
      //Select the next cloud in the list
      extractionManager->set_highlightON(false);
      sceneManager->select_nextMesh();
      break;
    }

    //Suppr key - Delete selected
    if (ImGui::IsKeyPressed(261)){
      bool atLeastOne = selectionManager->mark_supressSelectedPoints_all();

      if(atLeastOne == false){
        Mesh* mesh = sceneManager->get_selectedMesh();
        sceneManager->removeCloud(mesh);
      }
      break;
    }

    //Fin key - Delete all
    if (ImGui::IsKeyPressed(269)){
      sceneManager->removeCloud_all();
      break;
    }

    //R key - Reset
    if (ImGui::IsKeyPressed(82) && !io.WantCaptureMouse){
      opeManager->reset();
      break;
    }

    //H key - Heatmap
    if (ImGui::IsKeyPressed(72) && !io.WantCaptureMouse){
      heatmapManager->set_HeatMap(mesh);
      break;
    }

    //5 key - Centering
    if (ImGui::IsKeyPressed(325) && !io.WantCaptureMouse){
      if(!sceneManager->is_listMeshEmpty()){
        transformManager.make_centering(mesh);
        sceneManager->update_CloudPosition(mesh);
      }
      break;
    }

    //T key - Test button
    if(ImGui::IsKeyPressed(84) && !io.WantCaptureMouse){
      Segmentation segManager;
      segManager.algo(mesh);

      break;
    }

    //Enter - Validation
    if(ImGui::IsKeyPressed(257) && !io.WantCaptureMouse){
      selectionManager->validate();
      break;
    }

    //N key - Save all & remove all & load new
    if(ImGui::IsKeyPressed(78) && !io.WantCaptureMouse){
      opeManager->allSaving();
      sceneManager->removeCloud_all();
    }
  }

  //----------------------------
}
void GUI_control::control_keyboard_displace(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  ImGuiIO io = ImGui::GetIO();
  //----------------------------

  for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++){
    if(!io.MouseDown[1] && !io.WantCaptureMouse){
      //Shift speed up
      if(io.KeysDown[340]){
        transCoef = m_transCoef_fast;
      }else{
        transCoef = m_transCoef_slow;
      }
      // Z key
      if(io.KeysDown[87]){
        if(!sceneManager->is_listMeshEmpty()){
          vec3 translation = vec3(transCoef,0,0);
          transformManager.make_translation(mesh, translation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
      // S key
      if(io.KeysDown[83]){
        if(!sceneManager->is_listMeshEmpty()){
          vec3 translation = vec3(-transCoef,0,0);
          transformManager.make_translation(mesh, translation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
      // D key
      if(io.KeysDown[68]){
        if(!sceneManager->is_listMeshEmpty()){
          vec3 translation = vec3(0,transCoef,0);
          transformManager.make_translation(mesh, translation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
      // Q key
      if(io.KeysDown[65]){
        if(!sceneManager->is_listMeshEmpty()){
          vec3 translation = vec3(0,-transCoef,0);
          transformManager.make_translation(mesh, translation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
      // A key
      if(io.KeysDown[81]){
        if(!sceneManager->is_listMeshEmpty()){
          vec3 translation = vec3(0,0,transCoef);
          transformManager.make_translation(mesh, translation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
      // E key
      if(io.KeysDown[69]){
        if(!sceneManager->is_listMeshEmpty()){
          vec3 translation = vec3(0,0,-transCoef);
          transformManager.make_translation(mesh, translation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
      // 7 key
      if(io.KeysDown[327]){
        if(!sceneManager->is_listMeshEmpty()){
          float r = rotatDegree*M_PI/180;
          vec3 rotation = vec3(0,0,r);
          transformManager.make_rotation(mesh, vec3(0,0,0), rotation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
      // 9 key
      if(io.KeysDown[329]){
        if(!sceneManager->is_listMeshEmpty()){
          float r = rotatDegree*M_PI/180;
          vec3 rotation = vec3(0,0,-r);
          transformManager.make_rotation(mesh, vec3(0,0,0), rotation);
          sceneManager->update_CloudPosition(mesh);
        }
        break;
      }
    }
  }

  //----------------------------
}
void GUI_control::control_keyboard_ctrlAction(){
  ImGuiIO io = ImGui::GetIO();
  //----------------------------

  //if ctrl
  if(ImGui::IsKeyPressed(341)){
    //ctrl+s - Save as
    if(ImGui::IsKeyPressed(83)){
      opeManager->saving();
    }
    //ctrl+w - Open
    if(ImGui::IsKeyPressed(90)){
      opeManager->loading();
    }
    //ctrl+right click - move square
    if(io.MouseDownDuration[0] >= 0.01f && !io.WantCaptureMouse){
      glyphManager->obj_cube();
    }
  }

  //----------------------------
}
