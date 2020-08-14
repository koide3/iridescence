#ifndef GUIK_MODEL_CONTROL_HPP
#define GUIK_MODEL_CONTROL_HPP

#include <sstream>

#include <imgui.h>
#include <ImGuizmo.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace guik {

class ModelControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ModelControl(const std::string& name);

  void draw_ui();
  void draw_gizmo(int win_x, int win_y, int win_w, int win_h, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection, bool on_window=false);

  Eigen::Matrix4f model_matrix() const;

private:
  std::string name;
  Eigen::Affine3f pose;

  ImGuizmo::OPERATION gizmo_operation;
};

}  // namespace guik

#endif