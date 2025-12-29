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
  void set_model_matrix(const Eigen::Matrix4d& matrix) { pose = Eigen::Affine3f(matrix.cast<float>()); }
  template <typename Scalar, int Mode>
  void set_model_matrix(const Eigen::Transform<Scalar, 3, Mode>& matrix) {
    pose = Eigen::Affine3f(matrix.template cast<float>());
  }

  void set_gizmo_enabled(bool enabled);
  void enable_gizmo();
  void disable_gizmo();

  // "TRANSLATE", "ROTATE", "SCALE", "SCALEU", or "UNIVERSAL"
  void set_gizmo_operation(const std::string& operation);
  void set_gizmo_operation(int operation);

  // ImGuizmo mode (LOCAL = 0, WORLD = 1)
  void set_gizmo_mode(int mode);

  // Change the gizmo size (default = 0.1f)
  void set_gizmo_clip_scale(float space = 0.1f);

private:
  std::string name;
  Eigen::Affine3f pose;

  bool gizmo_enabled;
  int gizmo_operation;
  int gizmo_mode;
  float gizmo_clip_space;
};

}  // namespace guik

#endif