#include "GUI_Option.h"
#include "GUI_Control.h"

#include "../Engine/Scene.h"
#include "../Engine/Engine.h"
#include "../Engine/Glyphs.h"
#include "../Operation/Operation.h"
#include "../Operation/Attribut.h"
#include "../Operation/Functions/Heatmap.h"
#include "../Operation/Transforms.h"

//Constructor / Destructor
GUI_option::GUI_option(Engine* engine, GUI_control* control){
  this->gui_controlManager = control;
  this->engineManager = engine;
  //---------------------------

  this->sceneManager = engineManager->get_SceneManager();
  this->heatmapManager = engineManager->get_heatmapManager();
  this->attribManager = engineManager->get_AttribManager();
  this->glyphManager = engineManager->get_glyphManager();
  this->opeManager = engineManager->get_OpeManager();

  //---------------------------
}
GUI_option::~GUI_option(){}

//Main function
void GUI_option::design_Options(){
  //---------------------------

  this->option_font();
  this->option_glyphs();
  this->option_heatmap();
  this->option_parameters();
  this->option_colors();

  //---------------------------
}

//Subfunctions
void GUI_option::option_glyphs(){
  Mesh* mesh = sceneManager->get_selectedMesh();
  ImGuiStyle& style = ImGui::GetStyle();
  //---------------------------

  //Display grid
  static bool gridON = true;
  if(ImGui::Checkbox("Grid", &gridON)){
    glyphManager->set_visibility("grid", gridON);
  }
  ImGui::SameLine();
  static bool subGridON = false;
  if(ImGui::Checkbox("Subgrid", &subGridON)){
    glyphManager->set_visibility("subgrid", subGridON);
    glyphManager->set_visibility("planegrid", subGridON);
  }

  //Display Bounding Box
  bool* aabbON = glyphManager->get_aabbVisibility();
  if(ImGui::Checkbox("AABB", aabbON)){
    glyphManager->set_visibility("aabb", *aabbON);
  }
  ImGui::SameLine();

  //Display normals
  static bool normalsON = false;
  if(ImGui::Checkbox("Normal", &normalsON)){
    glyphManager->set_visibility("normal", normalsON);
  }
  //Display ICP line correspondences
  bool* matchingON = glyphManager->get_matchVisibility();
  ImGui::Checkbox("Match", matchingON);
  ImGui::SameLine();

  //Display Axis
  static bool axisON = true;
  if(ImGui::Checkbox("Axis", &axisON)){
    glyphManager->set_visibility("axis", axisON);
    glyphManager->set_visibility("axisMesh", axisON);
  }

  //Visualization mode
  static bool visualizeON = false;
  if(ImGui::Checkbox("Visualization mode", &visualizeON)){
    this->option_visualizationMode(visualizeON);
  }

  //Axis circle
  static float circleRadius = 1;
  ImGui::PushItemWidth(50);
  if(ImGui::DragFloat("##456", &circleRadius, 0.001, 0, 5, "%.3f")){
    glyphManager->obj_axisCircle(circleRadius);
  }
  ImGui::SameLine();

  //Display Axis circle
  static bool axisCircleON = false;
  if(ImGui::Checkbox("Axis circle", &axisCircleON)){
    glyphManager->obj_axisCircle(circleRadius);
    glyphManager->set_visibility("axisCircle", axisCircleON);
  }

  //---------------------------
  ImGui::Separator();
}
void GUI_option::option_heatmap(){
  if(ImGui::CollapsingHeader("Heatmap")){
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    //HeatMap
    if(ImGui::Button("Heatmap", ImVec2(75,0))){
      if(sceneManager->is_atLeastOneMesh()){
        heatmapManager->set_HeatMap(mesh);
      }
    }
    ImGui::SameLine();

    ImGui::PushItemWidth(75);
    static int style_idx = 0;
    if (ImGui::Combo("##1", &style_idx, "Is\0dist\0cos(It)\0It\0")){
        heatmapManager->set_HeatMapField(style_idx);
    }
    if(ImGui::Button("Palette", ImVec2(75,0))){
      if(mesh->intensity.heatmap){
        heatmapManager->plot_colorPalette(mesh);
      }
    }
    ImGui::SameLine();

    //Normalize heatmap
    static bool normalizeON = false;
    if(ImGui::Checkbox("Normalized", &normalizeON)){
      heatmapManager->set_normalized(normalizeON);
    }

    //Heatmap all
    static bool heatAll = false;
    if(ImGui::Button("Apply all", ImVec2(75,0))){
      if(sceneManager->is_atLeastOneMesh()){
        heatAll = !heatAll;
        heatmapManager->set_HeatMap_all(heatAll);
      }
    }

    //---------------------------
    ImGui::Separator();
  }
}
void GUI_option::option_colors(){
  if(ImGui::CollapsingHeader("Colors")){
    Mesh* mesh = sceneManager->get_selectedMesh();
    //---------------------------

    //Background color
    ImGui::SetNextItemWidth(100);
    ImGui::ColorEdit3("Background", (float*)backgColor);

    //Normals color
    ImGui::SetNextItemWidth(100);
    static vec3 color_normals = glyphManager->get_normalColor();
    if(ImGui::ColorEdit3("Normals", (float*)&color_normals)){
      int ID = glyphManager->get_glyphID("normal");
      glyphManager->changeColor(ID, color_normals);
    }

    //Grid color
    ImGui::SetNextItemWidth(100);
    static vec3 color_grid = glyphManager->get_gridColor();
    if(ImGui::ColorEdit3("Grid", (float*)&color_grid)){
      int ID = glyphManager->get_glyphID("grid");
      glyphManager->changeColor(ID, color_grid);
    }

    //Bounding box color
    ImGui::SetNextItemWidth(100);
    static vec3 color_aabb = glyphManager->get_aabbColor();
    if(ImGui::ColorEdit3("AABB", (float*)&color_aabb)){
      int ID = glyphManager->get_glyphID("aabb");
      glyphManager->changeColor(ID, color_aabb);
    }

    //Uniform cloud color
    ImGui::SetNextItemWidth(100);
    static vec4 color_PC;
    if(ImGui::ColorEdit4("Point cloud", (float*)&color_PC, ImGuiColorEditFlags_AlphaBar)){
      if(sceneManager->is_atLeastOneMesh()){
        attribManager->set_pointCloudColor(mesh, color_PC);
      }
    }

    //Matching color
    ImGui::SetNextItemWidth(100);
    static vec3 color_matching = glyphManager->get_matchingColor();
    if(ImGui::ColorEdit3("Matching", (float*)&color_matching)){
      int ID = glyphManager->get_glyphID("matching");
      glyphManager->changeColor(ID, color_matching);
    }

    //---------------------------
    ImGui::Separator();
    if(ImGui::Button("Reset", ImVec2(75,0))){
      this->backgColor->x = configuration.WINDOW_BckgColor;
      this->backgColor->y = configuration.WINDOW_BckgColor;
      this->backgColor->z = configuration.WINDOW_BckgColor;
      glyphManager->reset_colors();
    }
  }
}
void GUI_option::option_parameters(){
  if(ImGui::CollapsingHeader("Parameters")){
    Mesh* mesh = sceneManager->get_selectedMesh();
    ImGuiStyle& style = ImGui::GetStyle();
    //---------------------------

    //Mesh movement
    float* transCoef = gui_controlManager->get_transCoef();
    ImGui::DragFloat("Translation", transCoef, 0.001, 0, 100, "%.4f");
    float* rotatDegree = gui_controlManager->get_rotatDegree();
    ImGui::DragFloat("Rotation", rotatDegree, 0.5, 0, 90, "%.4f");

    //Point cloud scaling
    static float scale = 1.0f;
    ImGui::SetNextItemWidth(100);
    if(ImGui::DragFloat("Scale", &scale, 0.01, 0.1, 10, "%.2f X")){
      if(sceneManager->is_atLeastOneMesh()){
        Transforms transformManager;
        transformManager.make_scaling(mesh, scale);
        sceneManager->update_CloudPosition(mesh);
      }
    }

    //Point size
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Points size:");
    ImGui::SameLine();
    static int cpt_pts = 1;
    float spacing = style.ItemInnerSpacing.x;
    ImGui::PushButtonRepeat(true);
    if (ImGui::ArrowButton("##left", ImGuiDir_Left)){
      cpt_pts--;
      if(cpt_pts<=1) cpt_pts=1;
      engineManager->set_pointSize(cpt_pts);
    }
    ImGui::SameLine(0.0f, spacing);
    if (ImGui::ArrowButton("##right", ImGuiDir_Right)){
      cpt_pts++;
      engineManager->set_pointSize(cpt_pts);
    }
    ImGui::PopButtonRepeat();
    ImGui::SameLine();
    ImGui::Text("%d", cpt_pts);

    //Normals size
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Normal size:");
    ImGui::SameLine();
    static int cpt_nor = 1;
    float spacing_n = style.ItemInnerSpacing.x;
    ImGui::PushButtonRepeat(true);
    if(ImGui::ArrowButton("##left_n", ImGuiDir_Left)){
      cpt_nor--;
      if(cpt_nor <= 1){
        cpt_nor = 1;
      }
      glyphManager->set_normalSize(cpt_nor);
      glyphManager->obj_normals(mesh);
    }
    ImGui::SameLine(0.0f, spacing_n);
    if(ImGui::ArrowButton("##right_n", ImGuiDir_Right)){
      cpt_nor++;
      glyphManager->set_normalSize(cpt_nor);
      glyphManager->obj_normals(mesh);
    }
    ImGui::PopButtonRepeat();
    ImGui::SameLine();
    ImGui::Text("%d", cpt_nor);

    //---------------------------
    ImGui::Separator();
  }
}
void GUI_option::option_visualizationMode(bool visualizeON){
  if(visualizeON == true){
    glyphManager->set_visibility("axis", false);
    glyphManager->set_visibility("axisMesh", false);
    glyphManager->set_visibility("grid", false);
    glyphManager->set_visibility("aabb", false);
    *backgColor = vec3(1.0f,1.0f,1.0f);
  }else{
    glyphManager->set_visibility("axis", true);
    glyphManager->set_visibility("axisMesh", true);
    glyphManager->set_visibility("grid", true);
    glyphManager->set_visibility("aabb", true);
    *backgColor = vec3(0,0,0);
  }
}
void GUI_option::option_font(){
  ImGuiIO& io = ImGui::GetIO();
  //---------------------------

  static int font_selected = 0;
  ImGui::PushItemWidth(50);
  if(ImGui::Combo("Font size", &font_selected, "12\0 13\0")){
    ImFont* font = io.Fonts->Fonts[font_selected];
    io.FontDefault = font;
  }

  //---------------------------
}
