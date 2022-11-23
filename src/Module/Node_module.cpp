#include "Node_module.h"

#include "../Engine/Node_engine.h"
#include "../Operation/Node_operation.h"
#include "../Interface/Node_interface.h"


//Constructor / destructor
Node_module::Node_module(Node_engine* node){
  //---------------------------


  //---------------------------
  this->load_module();
}
Node_module::~Node_module(){}

void Node_module::load_module(){
  //---------------------------



  //---------------------------
}
void Node_module::reset(){
  //---------------------------



  //---------------------------
}
void Node_module::update(){
  //---------------------------



  //---------------------------
}
void Node_module::runtime(){
  //---------------------------

  //---------------------------
}
void Node_module::draw(){
  if(ImGui::BeginTabItem("Module")){
    if(ImGui::BeginTabBar("##tabs_module", ImGuiTabBarFlags_None)){
      //---------------------------


      //---------------------------
      ImGui::EndTabBar();
    }
    ImGui::EndTabItem();
  }
}
void Node_module::draw_online(){
  //---------------------------



  //---------------------------
}
void Node_module::online(Cloud* cloud, int subset_ID){
  //---------------------------


  //---------------------------
}
