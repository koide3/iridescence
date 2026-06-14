#include <guik/camera/orbit_camera_control_xy.hpp>

#include <memory>
#include <iostream>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>

namespace guik {

OrbitCameraControlXY::OrbitCameraControlXY() : OrbitCameraControlXY(80.0, 0.0, -60.0f * M_PI / 180.0f) {}

OrbitCameraControlXY::OrbitCameraControlXY(double distance, double theta, double phi) {
  center_offset.setZero();
  center.setZero();
  this->distance = distance;
  this->theta = theta;
  this->phi = phi;

  left_button_down = false;
  right_button_down = false;
  middle_button_down = false;
}

OrbitCameraControlXY::~OrbitCameraControlXY() {}

void OrbitCameraControlXY::reset_center() {
  center.setZero();
}

void OrbitCameraControlXY::lookat(const Eigen::Vector3f& pt) {
  center_offset = pt;
}

void OrbitCameraControlXY::mouse(const Eigen::Vector2f& p, int button, bool down) {
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

void OrbitCameraControlXY::drag(const Eigen::Vector2f& p, int button) {
  Eigen::Vector2f rel = p - drag_last_pos;

  if (left_button_down && right_button_down) {
    center.z() += rel[1] * distance * 0.001f;
  } else if (left_button_down) {
    theta -= rel[0] * 0.01f;
    phi -= rel[1] * 0.01f;

    phi = std::min(M_PI_2 - 0.01, std::max(-M_PI_2 + 0.01, phi));
  } else if (middle_button_down || right_button_down) {
    center += Eigen::AngleAxisf(theta + M_PI_2, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(-rel[0], rel[1], 0.0f) * distance * 0.001f;
  }

  drag_last_pos = p;
}

void OrbitCameraControlXY::scroll(const Eigen::Vector2f& rel) {
  if (rel[0] > 0) {
    distance = distance * 0.8f;
  } else if (rel[0] < 0) {
    distance = distance * 1.2f;
  }

  distance = std::max(0.1, distance);
}

void OrbitCameraControlXY::updown(double p) {
  if (p > 0) {
    distance = distance * 0.998f;
  } else if (p < 0) {
    distance = distance * 1.002f;
  }

  distance = std::max(0.1, distance);
}

void OrbitCameraControlXY::arrow(const Eigen::Vector2f& p) {
  center += Eigen::AngleAxisf(theta + M_PI_2, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(-p[0], p[1], 0.0f) * distance * 0.0001f;
}

Eigen::Vector2f OrbitCameraControlXY::depth_range() const {
  return Eigen::Vector2f(distance / 10.0f, distance * 10.0f);
}

Eigen::Quaternionf OrbitCameraControlXY::rotation() const {
  return Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY());
}

Eigen::Matrix4f OrbitCameraControlXY::view_matrix() const {
  Eigen::Vector3f center_ = center_offset + center;
  Eigen::Vector3f eye_offset = rotation() * Eigen::Vector3f(distance, 0.0f, 0.0f);
  Eigen::Vector3f eye = center_ + eye_offset;
  Eigen::Vector3f up = Eigen::Vector3f::UnitZ();

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center_[0], center_[1], center_[2]), glm::vec3(up[0], up[1], up[2]));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
}

void OrbitCameraControlXY::load(std::istream& ist) {
  std::string token;
  ist >> token >> center_offset[0] >> center_offset[1] >> center_offset[2];
  ist >> token >> center[0] >> center[1] >> center[2];
  ist >> token >> distance;
  ist >> token >> theta;
  ist >> token >> phi;
}

void OrbitCameraControlXY::save(std::ostream& ost) const {
  ost << "center_offset: " << center_offset.transpose() << std::endl;
  ost << "center: " << center.transpose() << std::endl;
  ost << "distance: " << distance << std::endl;
  ost << "theta: " << theta << std::endl;
  ost << "phi: " << phi << std::endl;
}

}  // namespace guik
