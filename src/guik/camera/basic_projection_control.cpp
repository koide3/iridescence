#include <guik/camera/basic_projection_control.hpp>

#include <iostream>

#include <imgui.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace guik {

BasicProjectionControl::BasicProjectionControl(const Eigen::Vector2i& size) : size(size), projection_mode(0), fovy(30.0f), width(10.0f), near_(1.0f), far_(1000.0f) {}

BasicProjectionControl::~BasicProjectionControl() {}

void BasicProjectionControl::set_projection_mode(int mode) {
  this->projection_mode = mode;
}

void BasicProjectionControl::set_fovy(double fovy_deg) {
  this->fovy = fovy_deg;
}

void BasicProjectionControl::set_ortho_width(double width) {
  this->width = width;
}

void BasicProjectionControl::set_depth_range(const Eigen::Vector2f& range) {
  this->near_ = range[0];
  this->far_ = range[1];

  width = (this->far_ - this->near_) * 0.1;
}

Eigen::Matrix4f BasicProjectionControl::projection_matrix() const {
  double aspect_ratio = size[0] / static_cast<float>(size[1]);

  glm::mat4 proj;
  if (projection_mode == 0) {
    proj = glm::perspective<float>(fovy * M_PI / 180.0, aspect_ratio, near_, far_);
  } else {
    proj = glm::ortho<float>(-width / 2.0f, width / 2.0f, -width / 2.0f / aspect_ratio, width / 2.0 / aspect_ratio, near_, far_);
  }

  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));
}

void BasicProjectionControl::draw_ui() {
  const char* modes[] = {"PERSPECTIVE", "ORTHOGONAL"};
  ImGui::Combo("Mode", &projection_mode, modes, IM_ARRAYSIZE(modes));
  if (projection_mode == 0) {
    ImGui::DragFloat("FOV", &fovy, 0.1f, 1.0f, 180.0f);
  }
}

std::string BasicProjectionControl::name() const {
  const char* modes[] = {"PERSPECTIVE", "ORTHOGONAL"};
  return modes[projection_mode];
}

void BasicProjectionControl::load(std::istream& ist) {
  std::string token, mode;
  ist >> token >> mode;

  if (mode == "ORTHOGONAL") {
    projection_mode = 1;
  } else {
    projection_mode = 0;
  }

  std::cout << token << std::endl;
  ist >> token >> fovy;
  std::cout << token << std::endl;
  ist >> token >> width;
  std::cout << token << std::endl;
  ist >> token >> near_;
  std::cout << token << std::endl;
  ist >> token >> far_;
  std::cout << token << std::endl;
}

void BasicProjectionControl::save(std::ostream& ost) const {
  ost << "fovy: " << fovy << std::endl;
  ost << "width: " << width << std::endl;
  ost << "near: " << near_ << std::endl;
  ost << "far: " << far_ << std::endl;
}

}  // namespace guik