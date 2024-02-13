#include <guik/camera/arcball_camera_control.hpp>

#include <memory>
#include <iostream>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace guik {

/*
 * SO3 expmap code taken from Sophus
 * https://github.com/strasdat/Sophus/blob/593db47500ea1a2de5f0e6579c86147991509c59/sophus/so3.hpp#L585
 *
 * Copyright 2011-2017 Hauke Strasdat
 *           2012-2017 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
inline Eigen::Quaterniond so3_exp(const Eigen::Vector3d& omega) {
  double theta_sq = omega.dot(omega);

  double theta;
  double imag_factor;
  double real_factor;
  if(theta_sq < 1e-10) {
    theta = 0;
    double theta_quad = theta_sq * theta_sq;
    imag_factor = 0.5 - 1.0 / 48.0 * theta_sq + 1.0 / 3840.0 * theta_quad;
    real_factor = 1.0 - 1.0 / 8.0 * theta_sq + 1.0 / 384.0 * theta_quad;
  } else {
    theta = std::sqrt(theta_sq);
    double half_theta = 0.5 * theta;
    imag_factor = std::sin(half_theta) / theta;
    real_factor = std::cos(half_theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());
}

ArcBallCameraControl::ArcBallCameraControl() : ArcBallCameraControl(80.0) {}

ArcBallCameraControl::ArcBallCameraControl(double distance, const Eigen::Quaternionf& quat) {
  this->distance = distance;

  center.setZero();
  center_offset.setZero();

  delta_trans.setZero();
  orientation = quat;
  delta_orientation.setIdentity();

  left_button_down = false;
  middle_button_down = false;
  mouse_down_pos.setConstant(-1);
  drag_last_pos.setConstant(-1);
}

ArcBallCameraControl::~ArcBallCameraControl() {}

void ArcBallCameraControl::reset_center() {
  center.setZero();
}

void ArcBallCameraControl::lookat(const Eigen::Vector3f& pt) {
  center_offset = pt;
}

void ArcBallCameraControl::mouse(const Eigen::Vector2f& p, int button, bool down) {
  if(button == 0) {
    left_button_down = down;
  }
  if(button == 2) {
    middle_button_down = down;
  }

  if(down) {
    mouse_down_pos = p;
  } else {
    center += delta_trans;
    delta_trans.setZero();

    orientation = orientation * delta_orientation;
    delta_orientation.setIdentity();
  }

  drag_last_pos = p;
}

void ArcBallCameraControl::drag(const Eigen::Vector2f& p, int button) {
  if(left_button_down) {
    Eigen::Vector2f diff = p - mouse_down_pos;
    Eigen::Vector2d vel = diff.cast<double>() / 1024.0;

    delta_orientation = so3_exp(Eigen::Vector3d(0.0, -vel[1], -vel[0])).cast<float>();
  }

  if(middle_button_down) {
    Eigen::Vector2f diff = p - mouse_down_pos;
    Eigen::Vector2d vel = diff.cast<double>() / 1024.0;

    delta_trans = orientation * Eigen::Vector3f(0.0f, -vel[0], vel[1]) * distance / 5.0;
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

void ArcBallCameraControl::updown(double p) {
  if(p > 0) {
    distance = distance * 0.998f;
  } else if(p < 0) {
    distance = distance * 1.002f;
  }

  distance = std::max(0.1, distance);
}

void ArcBallCameraControl::arrow(const Eigen::Vector2f& p) {
  center += orientation * Eigen::Vector3f(0.0, -p[0], p[1]) * distance * 1e-5;
}

Eigen::Vector2f ArcBallCameraControl::depth_range() const {
  return Eigen::Vector2f(distance / 10.0f, distance * 10.0f);
}

Eigen::Matrix4f ArcBallCameraControl::view_matrix() const {
  Eigen::Vector3f center_ = center_offset + center + delta_trans;
  Eigen::Quaternionf orientation_ = orientation * delta_orientation;

  Eigen::Vector3f eye_offset = orientation_ * Eigen::Vector3f(distance, 0.0f, 0.0f);
  Eigen::Vector3f eye = center_ + eye_offset;
  Eigen::Vector3f up = orientation_ * Eigen::Vector3f::UnitZ();

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center_[0], center_[1], center_[2]), glm::vec3(up[0], up[1], up[2]));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
}

void ArcBallCameraControl::load(std::istream& ist) {
  std::string token;
  ist >> token >> center_offset.x() >> center_offset.y() >> center_offset.z();
  ist >> token >> center.x() >> center.y() >> center.z();
  ist >> token >> distance;
  ist >> token >> orientation.x() >> orientation.y() >> orientation.z() >> orientation.w();
}

void ArcBallCameraControl::save(std::ostream& ost) const {
  ost << "center_offset: " << center_offset.transpose() << std::endl;
  ost << "center: " << center.transpose() << std::endl;
  ost << "distance: " << distance << std::endl;
  ost << "orientation: " << orientation.x() << " " << orientation.y() << " " << orientation.z() << " " << orientation.w() << std::endl;
}

}  // namespace guik
