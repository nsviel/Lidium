#include "CoordTransform.h"

#include "../../Engine/OpenGL/Camera.h"
#include "../../Engine/Dimension.h"

//Constructor / Destructor
CoordTransform::CoordTransform(Camera* controls, Dimension* dim){
  this->cameraManager = controls;
  this->dimManager = dim;
  //---------------------------

  //---------------------------
}
CoordTransform::~CoordTransform(){}

vec2 CoordTransform::WorldToCursor(vec3 point){
  mat4 projMat = cameraManager->get_projMat();
  mat4 viewMat = cameraManager->get_viewMat();
  vec2 glDim = dimManager->get_glDim();
  vec2 pt_out;
  //---------------------------

  vec4 viewport(0, 0, glDim.x, glDim.y);

  vec3 projected = glm::project(point, viewMat, projMat, viewport);
  pt_out.x = projected.x;
  pt_out.y = glDim.y - projected.y;

  //---------------------------
  return pt_out;
}
vec3 CoordTransform::CursorToWorld(vec2 cursorPos){
  vec2 glDim = dimManager->get_glDim();
  float gui_X = ImGui::GetWindowSize().x;
  float gui_Y = ImGui::GetWindowSize().y;
  float gl_X = glDim.x;
  float gl_Y = glDim.y;
  //---------------------------

  //Get ray direction
  float x = (2.0f * cursorPos.x) / gl_X - 1.0f;
  float y = 1.0f - (2.0f * cursorPos.y) / gl_Y;
  float z = 1.0f;
  vec3 ray_nds = vec3(x, y, z);
  vec4 ray_clip = vec4(ray_nds.x, ray_nds.y, -1.0, 1.0);
  mat4 projMat = cameraManager->get_projMat();
  vec4 ray_eye = inverse(projMat) * ray_clip;
  ray_eye = vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);
  mat4 viewMat = cameraManager->get_viewMat();
  vec4 ray_wor = inverse(viewMat) * ray_eye;
  vec3 ray_world = vec3(ray_wor);
  vec3 ray_dir = normalize(ray_world);

  vec3 cam_pos = cameraManager->get_camPos();
  vec3 cam_forw = cameraManager->get_camForward();
  vec3 frustum = cam_pos + vec3(cam_forw.x*0.5,cam_forw.y*0.5,cam_forw.z*0.5);

  vec3 D = ray_dir;
  vec3 N = cam_forw;
  vec3 X = cam_pos + D * dot(frustum - cam_pos, N) / dot(D, N);

  //---------------------------
  return X;
}
vec3 CoordTransform::CursorToGround(){
  vec2 cursorPos = dimManager->get_cursorPos();
  vec2 glDim = dimManager->get_glDim();
  float gui_X = ImGui::GetWindowSize().x;
  float gui_Y = ImGui::GetWindowSize().y;
  float gl_X = glDim.x;
  float gl_Y = glDim.y;
  //---------------------------

  //Get ray direction
  float x = (2.0f * cursorPos.x) / gl_X - 1.0f;
  float y = 1.0f - (2.0f * cursorPos.y) / gl_Y;
  float z = 1.0f;
  vec3 ray_nds = vec3(x, y, z);
  vec4 ray_clip = vec4(ray_nds.x, ray_nds.y, -1.0, 1.0);
  mat4 projMat = cameraManager->get_projMat();
  vec4 ray_eye = inverse(projMat) * ray_clip;
  ray_eye = vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);
  mat4 viewMat = cameraManager->get_viewMat();
  vec4 ray_wor = inverse(viewMat) * ray_eye;
  vec3 ray_world = vec3(ray_wor);
  vec3 ray_dir = normalize(ray_world);

  vec3 cam_pos = cameraManager->get_camPos();
  vec3 P0 = vec3(0,0,0);
  vec3 P1 = vec3(1,0,0);
  vec3 P2 = vec3(0,1,0);

  //Obtain world position at z=0
  vec3 D = ray_dir;
  vec3 N = cross(P1-P0, P2-P0);
  vec3 X = cam_pos + D * dot(P0 - cam_pos, N) / dot(D, N);

  //---------------------------
  return X;
}
