#include <guik/camera/fps_camera_control.hpp>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>

namespace guik {

FPSCameraControl::FPSCameraControl(const Eigen::Vector2i& canvas_size)
: pos(0.0f, 0.0f, 1.0),
  pitch(0.0),
  yaw(0.0),
  size(canvas_size),
  fovy(60.0),
  near(0.1),
  far(1000.0),
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

void FPSCameraControl::set_depth_range(const Eigen::Vector2f& range) {}

Eigen::Matrix4f FPSCameraControl::projection_matrix() const {
  double aspect_ratio = size[0] / static_cast<float>(size[1]);

  glm::mat4 proj = glm::perspective<float>(fovy * M_PI / 180.0, aspect_ratio, near, far);

  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));
}

void FPSCameraControl::reset_center() {}

void FPSCameraControl::lookat(const Eigen::Vector3f& pt) {
  const Eigen::Vector3f view_vector = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitX();
  pos = pt - 3.0f * view_vector;
}

void FPSCameraControl::mouse(const Eigen::Vector2i& p, int button, bool down) {
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

void FPSCameraControl::drag(const Eigen::Vector2i& p, int button) {
  Eigen::Vector2i rel = p - drag_last_pos;
  if (left_button_down) {
    yaw -= rel[0] * mouse_sensitivity_yaw;
    pitch -= rel[1] * mouse_sensitivity_pitch;
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
  fovy -= rel[0];
  fovy = std::max(1.0, std::min(170.0, fovy));
}

void FPSCameraControl::updown(int p) {
  const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();
  pos += rotation.col(2) * p * translation_speed * 0.1;
}

void FPSCameraControl::arrow(const Eigen::Vector2i& p) {
  const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();
  yaw += p[0] * mouse_sensitivity_yaw * 0.5;
  pos += rotation.col(0) * p[1] * translation_speed * 0.1;
}

void FPSCameraControl::update() {
  const auto& io = ImGui::GetIO();

  const Eigen::Matrix3f rotation = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY())).toRotationMatrix();

  if (io.KeysDown[GLFW_KEY_W]) {
    pos += rotation.col(0) * translation_speed;
  }
  if (io.KeysDown[GLFW_KEY_S]) {
    pos -= rotation.col(0) * translation_speed;
  }
  if (io.KeysDown[GLFW_KEY_D]) {
    pos -= rotation.col(1) * translation_speed;
  }
  if (io.KeysDown[GLFW_KEY_A]) {
    pos += rotation.col(1) * translation_speed;
  }
  if (io.KeysDown[GLFW_KEY_R]) {
    pos += rotation.col(2) * translation_speed;
  }
  if (io.KeysDown[GLFW_KEY_F]) {
    pos -= rotation.col(2) * translation_speed;
  }
  if (io.KeysDown[GLFW_KEY_E]) {
    yaw -= mouse_sensitivity_yaw * 2.0;
  }
  if (io.KeysDown[GLFW_KEY_Q]) {
    yaw += mouse_sensitivity_yaw * 2.0;
  }
}

Eigen::Vector2f FPSCameraControl::depth_range() const {
  return Eigen::Vector2f(near,  far);
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