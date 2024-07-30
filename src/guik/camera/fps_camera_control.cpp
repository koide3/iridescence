#include <guik/camera/fps_camera_control.hpp>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>

namespace guik {

FPSCameraControl::FPSCameraControl(const Eigen::Vector2i& canvas_size)
: pos(0.0f, 0.0f, 1.0),
  yaw(0.0),
  pitch(0.0),
  min_fovy(1.0),
  max_fovy(170.0),
  default_fovy(60.0),
  fovy_locked(false),
  size(canvas_size),
  fovy(default_fovy),
  near_(0.1),
  far_(1000.0),
  mouse_sensitivity_yaw(0.01),
  mouse_sensitivity_pitch(0.01),
  translation_speed(0.1),
  left_button_down(false),
  right_button_down(false),
  middle_button_down(false),
  drag_last_pos(0, 0) {}

FPSCameraControl::~FPSCameraControl() {}

void FPSCameraControl::set_mouse_senstivity(double mouse_sensitivity_pitch, double mouse_sensitivity_yaw) {
  this->mouse_sensitivity_pitch = mouse_sensitivity_pitch;
  this->mouse_sensitivity_yaw = mouse_sensitivity_yaw;
}

void FPSCameraControl::set_translation_speed(double speed) {
  translation_speed = speed;
}

void FPSCameraControl::set_size(const Eigen::Vector2i& size) {
  this->size = size;
}

void FPSCameraControl::set_fovy_range(double min_fovy, double max_fovy, double default_fovy) {
  this->min_fovy = min_fovy;
  this->max_fovy = max_fovy;
  this->default_fovy = default_fovy;
}

void FPSCameraControl::reset_fovy() {
  this->fovy = this->default_fovy;
}

void FPSCameraControl::set_fovy(double fovy) {
  this->fovy = fovy;
}

void FPSCameraControl::lock_fovy() {
  fovy_locked = true;
}

void FPSCameraControl::unlock_fovy() {
  fovy_locked = false;
}

void FPSCameraControl::set_depth_range(const Eigen::Vector2f& range) {}

Eigen::Matrix4f FPSCameraControl::projection_matrix() const {
  double aspect_ratio = size[0] / static_cast<float>(size[1]);

  glm::mat4 proj = glm::perspective<float>(fovy * M_PI / 180.0, aspect_ratio, near_, far_);

  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));
}

void FPSCameraControl::set_pose(const Eigen::Vector3f& pos, double yaw_deg, double pitch_deg) {
  this->pos = pos;
  this->yaw = yaw_deg * M_PI / 180.0;
  this->pitch = pitch_deg * M_PI / 180.0;
}

void FPSCameraControl::reset_center() {}

void FPSCameraControl::lookat(const Eigen::Vector3f& pt) {
  const Eigen::Vector3f view_vector = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitX();
  pos = pt - 3.0f * view_vector;
}

void FPSCameraControl::mouse(const Eigen::Vector2f& p, int button, bool down) {
  if (button == 0) {
    left_button_down = down;
  }
  if (button == 1) {
    right_button_down = down;
  }
  if (button == 2) {
    middle_button_down = down;
  }

  drag_last_pos = p;
}

void FPSCameraControl::drag(const Eigen::Vector2f& p, int button) {
  Eigen::Vector2f rel = p - drag_last_pos;
  if (left_button_down) {
    yaw -= rel[0] * mouse_sensitivity_yaw;
    pitch -= rel[1] * mouse_sensitivity_pitch;
    pitch = std::max(-M_PI_2 + 1e-3, std::min(M_PI_2 - 1e-3, pitch));
  }

  if (right_button_down) {
    const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();
    pos -= rotation.col(2) * rel[1] * translation_speed * 0.1;
  }

  if (middle_button_down) {
    const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();
    pos -= rotation.col(1) * rel[0] * translation_speed * 0.1;
    pos -= rotation.col(0) * rel[1] * translation_speed * 0.1;
  }

  drag_last_pos = p;
}

void FPSCameraControl::scroll(const Eigen::Vector2f& rel) {
  if (fovy_locked) {
    return;
  }
  fovy -= rel[0];
  fovy = std::max(min_fovy, std::min(max_fovy, fovy));
}

void FPSCameraControl::updown(double p) {
  const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();
  pos += rotation.col(2) * p * translation_speed * 0.1;
}

void FPSCameraControl::arrow(const Eigen::Vector2f& p) {
  const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();
  yaw += p[0] * mouse_sensitivity_yaw * 0.5;
  pos += rotation.col(0) * p[1] * translation_speed * 0.1;
}

void FPSCameraControl::update() {
  const auto& io = ImGui::GetIO();
  const double dt = io.DeltaTime * 60.0;

  const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();

  if (io.KeysDown[GLFW_KEY_W]) {
    pos += rotation.col(0) * translation_speed * dt;
  }
  if (io.KeysDown[GLFW_KEY_S]) {
    pos -= rotation.col(0) * translation_speed * dt;
  }
  if (io.KeysDown[GLFW_KEY_D]) {
    pos -= rotation.col(1) * translation_speed * dt;
  }
  if (io.KeysDown[GLFW_KEY_A]) {
    pos += rotation.col(1) * translation_speed * dt;
  }
  if (io.KeysDown[GLFW_KEY_R]) {
    pos += rotation.col(2) * translation_speed * dt;
  }
  if (io.KeysDown[GLFW_KEY_F]) {
    pos -= rotation.col(2) * translation_speed * dt;
  }
  if (io.KeysDown[GLFW_KEY_E]) {
    yaw -= mouse_sensitivity_yaw * 2.0 * dt;
  }
  if (io.KeysDown[GLFW_KEY_Q]) {
    yaw += mouse_sensitivity_yaw * 2.0 * dt;
  }
}

Eigen::Vector2f FPSCameraControl::depth_range() const {
  return Eigen::Vector2f(near_,  far_);
}

Eigen::Matrix4f FPSCameraControl::view_matrix() const {
  const Eigen::Vector3f eye = pos;
  const Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f view_vector = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitX();
  const Eigen::Vector3f center = eye + view_vector;

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center[0], center[1], center[2]), glm::vec3(up[0], up[1], up[2]));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
}
}
