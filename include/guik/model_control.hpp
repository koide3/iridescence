#ifndef GUIK_MODEL_CONTROL_HPP
#define GUIK_MODEL_CONTROL_HPP

#include <sstream>

#include <imgui.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace guik {

class ModelControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ModelControl(const std::string& name, const Eigen::Matrix4f& init_model_matrix = Eigen::Matrix4f::Identity());

  void draw_ui();
  void draw_gizmo_ui();
  void draw_gizmo();
  void draw_gizmo(int win_x, int win_y, int win_w, int win_h, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection, bool on_window = false);

  bool is_guizmo_using() const;
  
  const std::string& model_name() const;
  Eigen::Matrix4f model_matrix() const;
  void set_model_matrix(const Eigen::Matrix4f& matrix) { pose = Eigen::Affine3f(matrix); }

private:
  std::string name;
  Eigen::Affine3f pose;

  int gizmo_operation;
};

}  // namespace guik

#endif