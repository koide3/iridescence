#include <guik/camera/topdown_camera_control.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace guik {

TopDownCameraControl::TopDownCameraControl() : TopDownCameraControl(80.0, 0.0) {}

TopDownCameraControl::TopDownCameraControl(double distance, double theta) {
  center.setZero();
  center_offset.setZero();
  this->distance = distance;
  this->theta = theta;

  left_button_down = false;
  middle_button_down = false;
}

TopDownCameraControl::~TopDownCameraControl() {}

void TopDownCameraControl::reset_center() {
  center.setZero();
}

void TopDownCameraControl::lookat(const Eigen::Vector3f& pt) {
  center_offset = pt;
}

void TopDownCameraControl::mouse(const Eigen::Vector2f& p, int button, bool down) {
  if(button == 0) {
    left_button_down = down;
  }
  if(button == 2) {
    middle_button_down = down;
  }
  drag_last_pos = p;
}

void TopDownCameraControl::drag(const Eigen::Vector2f& p, int button) {
  Eigen::Vector2f rel = p - drag_last_pos;

  if(left_button_down) {
    theta -= rel[0] * 0.01f;
  }

  if(middle_button_down) {
    center += Eigen::AngleAxisf(theta + M_PI_2, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(rel[0], -rel[1], 0.0f) * distance * 0.0005f;
  }

  drag_last_pos = p;
}

void TopDownCameraControl::scroll(const Eigen::Vector2f& rel) {
  if(rel[0] > 0) {
    distance = distance * 0.8f;
  } else if(rel[0] < 0) {
    distance = distance * 1.2f;
  }

  distance = std::max(0.1, distance);
}

Eigen::Vector2f TopDownCameraControl::depth_range() const {
  return Eigen::Vector2f(distance / 10.0f, distance * 10.0f);
}

Eigen::Matrix4f TopDownCameraControl::view_matrix() const {
  Eigen::Vector3f center_ = center_offset + center;
  Eigen::Vector3f eye = center_ + Eigen::Vector3f::UnitZ() * distance;
  Eigen::Vector3f up = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f::UnitX();

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center_[0], center_[1], center_[2]), glm::vec3(up[0], up[1], up[2]));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
}

void TopDownCameraControl::load(std::istream& ist) {
  std::string token;
  ist >> token >> center_offset[0] >> center_offset[1] >> center_offset[2];
  ist >> token >> center[0] >> center[1] >> center[2];
  ist >> token >> distance;
  ist >> token >> theta;
}

void TopDownCameraControl::save(std::ostream& ost) const {
  ost << "center_offset: " << center_offset.transpose() << std::endl;
  ost << "center: " << center.transpose() << std::endl;
  ost << "distance: " << distance << std::endl;
  ost << "theta: " << theta << std::endl;
}

}