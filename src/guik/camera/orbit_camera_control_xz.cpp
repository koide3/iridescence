#include <guik/camera/orbit_camera_control_xz.hpp>

#include <memory>
#include <iostream>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>

namespace guik {

OrbitCameraControlXZ::OrbitCameraControlXZ() : OrbitCameraControlXY() {
  theta = 0.0f;
  phi = 0.0f;
}

OrbitCameraControlXZ::OrbitCameraControlXZ(double distance, double theta, double phi) : OrbitCameraControlXY() {
  this->distance = distance;
  this->theta = theta;
  this->phi = phi;
}

OrbitCameraControlXZ::~OrbitCameraControlXZ() {}

void OrbitCameraControlXZ::drag(const Eigen::Vector2f& p, int button) {
  Eigen::Vector2f rel = p - drag_last_pos;

  if (left_button_down && right_button_down) {
    center.y() -= rel[1] * distance * 0.001f;
  } else if (left_button_down) {
    theta -= rel[0] * 0.01f;
    phi -= rel[1] * 0.01f;

    phi = std::min(M_PI_2 - 0.01, std::max(-M_PI_2 + 0.01, phi));
  } else if (middle_button_down || right_button_down) {
    center += Eigen::AngleAxisf(theta + M_PI_2, -Eigen::Vector3f::UnitY()) * Eigen::Vector3f(rel[1], 0.0f, rel[0]) * distance * 0.001f;
  }

  drag_last_pos = p;
}

Eigen::Quaternionf OrbitCameraControlXZ::rotation() const {
  return Eigen::AngleAxisf(theta, -Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX());
}

Eigen::Matrix4f OrbitCameraControlXZ::view_matrix() const {
  Eigen::Vector3f center_ = center_offset + center;
  Eigen::Vector3f eye_offset = rotation() * Eigen::Vector3f(0.0f, 0.0f, -distance);
  Eigen::Vector3f eye = center_ + eye_offset;
  Eigen::Vector3f up = -Eigen::Vector3f::UnitY();

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center_[0], center_[1], center_[2]), glm::vec3(up[0], up[1], up[2]));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
}

void OrbitCameraControlXZ::load(std::istream& ist) {
  OrbitCameraControlXY::load(ist);
}

void OrbitCameraControlXZ::save(std::ostream& ost) const {
  OrbitCameraControlXY::save(ost);
}

}  // namespace guik
