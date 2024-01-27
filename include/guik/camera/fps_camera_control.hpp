#ifndef GUIK_FPS_CAMERA_CONTROL_HPP
#define GUIK_FPS_CAMERA_CONTROL_HPP

#include <guik/camera/camera_control.hpp>
#include <guik/camera/projection_control.hpp>

namespace guik {

class FPSCameraControl : public CameraControl, public ProjectionControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FPSCameraControl(const Eigen::Vector2i& canvas_size);
  ~FPSCameraControl() override;

  void set_mouse_senstivity(double mouse_sensitivity_pitch = 0.01, double mouse_sensitivity_yaw = 0.01);
  void set_translation_speed(double speed = 0.05);

  // ProjectionControl
  void set_size(const Eigen::Vector2i& size) override;
  void set_depth_range(const Eigen::Vector2f& range) override;
  Eigen::Matrix4f projection_matrix() const override;

  // ViewMatrixControl
  void reset_center() override;
  void lookat(const Eigen::Vector3f& pt) override;

  void mouse(const Eigen::Vector2i& p, int button, bool down) override;
  void drag(const Eigen::Vector2i& p, int button) override;
  void scroll(const Eigen::Vector2f& rel) override;

  void updown(int p) override;
  void arrow(const Eigen::Vector2i& p) override;
  void update() override;

  Eigen::Vector2f depth_range() const override;
  Eigen::Matrix4f view_matrix() const override;

private:
  Eigen::Vector3f pos;
  double yaw;
  double pitch;
  double distance;

  Eigen::Vector2i size;
  double fovy;
  double near;
  double far;

  double mouse_sensitivity_yaw;
  double mouse_sensitivity_pitch;
  double translation_speed;

  bool left_button_down;
  bool right_button_down;
  bool middle_button_down;
  Eigen::Vector2i drag_last_pos;
};

}  // namespace  guik

#endif
