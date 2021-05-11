#include <guik/camera/arcball_camera_control.hpp>

#include <memory>
#include <iostream>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace guik {

ArcBallCameraControl::ArcBallCameraControl() : ArcBallCameraControl(80.0) {}

ArcBallCameraControl::ArcBallCameraControl(double distance) {
  this->distance = distance;

  center.setZero();
  center_offset.setZero();

  orientation.setIdentity();
  delta_orientation.setIdentity();

  left_button_down = false;
  middle_button_down = false;
  drag_last_pos.setConstant(-1);
}

ArcBallCameraControl::~ArcBallCameraControl() {}

void ArcBallCameraControl::reset_center() {
  center.setZero();
}

void ArcBallCameraControl::lookat(const Eigen::Vector3f& pt) {
  center_offset = pt;
}

void ArcBallCameraControl::mouse(const Eigen::Vector2i& p, int button, bool down) {
  if(button == 0) {
    left_button_down = down;
  }
  if(button == 2) {
    middle_button_down = down;
  }
  drag_last_pos = p;
}

void ArcBallCameraControl::drag(const Eigen::Vector2i& p, int button) {
  Eigen::Vector2i rel = p - drag_last_pos;

  if(left_button_down && !rel.isZero()) {
    Eigen::Vector2f delta = rel.cast<float>() * 0.01f;

    Eigen::Vector3f up = orientation * Eigen::Vector3f::UnitZ();
    Eigen::Vector3f right = orientation * Eigen::Vector3f::UnitY();

    Eigen::Vector3f axis = -(delta.x() * up + delta.y() * right).normalized();
    orientation = orientation * Eigen::AngleAxisf(delta.norm(), axis);
  }

  drag_last_pos = p;
}

void ArcBallCameraControl::scroll(const Eigen::Vector2f& rel) {
  if(rel[0] > 0) {
    distance = distance * 0.8f;
  } else if(rel[0] < 0) {
    distance = distance * 1.2f;
  }

  distance = std::max(0.1, distance);
}

void ArcBallCameraControl::updown(int p) {
  if(p > 0) {
    distance = distance * 0.998f;
  } else if(p < 0) {
    distance = distance * 1.002f;
  }

  distance = std::max(0.1, distance);
}

void ArcBallCameraControl::arrow(const Eigen::Vector2i& p) {}

Eigen::Vector2f ArcBallCameraControl::depth_range() const {
  return Eigen::Vector2f(distance / 10.0f, distance * 10.0f);
}

Eigen::Matrix4f ArcBallCameraControl::view_matrix() const {
  Eigen::Vector3f center_ = center_offset + center;
  Eigen::Quaternionf orientation_ = orientation * delta_orientation;

  Eigen::Vector3f eye_offset = orientation_ * Eigen::Vector3f(distance, 0.0f, 0.0f);
  Eigen::Vector3f eye = center_ + eye_offset;
  Eigen::Vector3f up = orientation_ * Eigen::Vector3f::UnitZ();

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center_[0], center_[1], center_[2]), glm::vec3(up[0], up[1], up[2]));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
}

}  // namespace guik
