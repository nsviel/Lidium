#include "Followup.h"
#include "Camera.h"

#include "../../Node_engine.h"
#include "../../Scene/Scene.h"
#include "../../Scene/Configuration.h"

#include "../../../Specific/fct_maths.h"
#include "../../../Specific/fct_transtypage.h"


//Constructor / Destructor
Followup::Followup(Node_engine* node){
  //---------------------------

  this->configManager = node->get_configManager();
  this->sceneManager = node->get_sceneManager();
  this->cameraManager = node->get_cameraManager();

  //---------------------------
  this->update_configuration();
}
Followup::~Followup(){}

//Main function
void Followup::update_configuration(){
  //---------------------------

  this->camera_moved_trans = vec2(0, 0);
  this->camera_moved_rotat = 0;
  this->camera_distFromPos = 5;
  this->camera_nb_pose = 5;

  this->with_camera_follow = configManager->parse_json_b("module", "with_camera_follow");
  this->with_camera_absolute = false;
  this->with_camera_top = false;
  this->with_camera_root = false;

  //---------------------------
}
void Followup::camera_followUp(Cloud* cloud, int ID_subset){
  //---------------------------

  //Follow up camera
  if(with_camera_follow){
    this->camera_displacment(cloud, ID_subset);
  }

  //Ortho top view option
  if(with_camera_top){
    cameraManager->input_set_projection(2);
  }else{
    cameraManager->input_set_projection(0);
  }

  //---------------------------
}
void Followup::camera_mode(string mode){
  //---------------------------

  if(mode == "top"){
    cameraManager->input_set_view(0);
  }
  else if(mode == "oblique"){
    cameraManager->input_set_view(1);
  }

  //---------------------------
}

//Subfunctions
void Followup::camera_displacment(Cloud* cloud, int ID_subset){
  Subset* subset = sceneManager->get_subset_byID(cloud, ID_subset);
  if(subset == nullptr) return;
  Frame* frame_m0 = &subset->frame;
  //---------------------------

  if(frame_m0->ID >= camera_nb_pose){
    //Camera payload
    vec3 E = camera_payload(cloud, ID_subset);

    //Camera pose
    this->camera_position(subset, E);

    //Camera orientation
    this->camera_orientation(subset, E);
  }

  //---------------------------
}
vec3 Followup::camera_payload(Cloud* cloud, int ID_subset){
  vec3 E = vec3(0, 0, 0);
  //---------------------------

  //Check if no stationary
  Frame* frame_m0 = sceneManager->get_frame_byID(cloud, ID_subset);
  Frame* frame_m1 = sceneManager->get_frame_byID(cloud, ID_subset-1);
  vec3 pos_m0 = eigen_to_glm_vec3(frame_m0->trans_b);
  vec3 pos_m1 = eigen_to_glm_vec3(frame_m1->trans_b);
  float pos_dist = fct_distance(pos_m0, pos_m1);
  if(pos_dist < 0.1) return E;

  //Retrieve the mean of some previous pose
  for(int i=0; i<camera_nb_pose; i++){
    Frame* frame = sceneManager->get_frame_byID(cloud, ID_subset - i);
    vec3 pos = eigen_to_glm_vec3(frame->trans_b);

    for(int j=0; j<3; j++){
      E[j] += pos[j];
    }
  }
  for(int j=0; j<3; j++){
    E[j] = E[j] / camera_nb_pose;
  }

  //---------------------------
  return E;
}
void Followup::camera_position(Subset* subset, vec3 E){
  Frame* frame = &subset->frame;
  //---------------------------

  //Camera pose
  vec3 pose = eigen_to_glm_vec3(frame->trans_b);
  vec3 C = pose - camera_distFromPos * (pose - E);

  //Forced absolute camera position
  if(with_camera_absolute){
    vec3 camPos = cameraManager->get_camPos();
    vec3 camPos_new = vec3(C.x, C.y, camPos.z);
    cameraManager->set_cameraPos(camPos_new);
  }
  //Forced relative camera position
  else{
    vec3 camPos = cameraManager->get_camPos();

    float x = camPos.x + pose[0] - camera_moved_trans.x;
    float y = camPos.y + pose[1] - camera_moved_trans.y;
    float z = camPos.z;

    vec3 camPos_new = vec3(x, y, z);

    this->camera_moved_trans = vec3(pose[0], pose[1], 0);
    cameraManager->set_cameraPos(camPos_new);
  }

  //---------------------------
}
void Followup::camera_orientation(Subset* subset, vec3 E){
  Frame* frame = &subset->frame;
  //---------------------------

  //Forced camera angle
  if(with_camera_absolute){
    vec3 pose = eigen_to_glm_vec3(frame->trans_b);
    vec3 C = pose - camera_distFromPos * (pose - E);
    vec3 D = pose - C;
    float angle = atan(D.y, D.x) - atan(0.0f, 1.0f);

    cameraManager->set_angle_azimuth(angle);
  }
  //Relative camera angle
  else{

  }

  //---------------------------
}
void Followup::camera_reset(){
  //---------------------------

  this->camera_moved_trans = vec3(0, 0, 0);

  //---------------------------
}
