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

  // FOV
  void set_fovy_range(double min_fovy_deg, double max_fovy_deg, double default_fovy_deg = 60.0);
  void reset_fovy();
  void lock_fovy();
  void unlock_fovy();
  void set_fovy(double fovy_deg);

  void set_depth_range(const Eigen::Vector2f& range) override;
  Eigen::Matrix4f projection_matrix() const override;

  // ViewMatrixControl
  void set_pose(const Eigen::Vector3f& pos, double yaw_deg, double pitch_deg);
  void reset_center() override;
  void lookat(const Eigen::Vector3f& pt) override;

  void mouse(const Eigen::Vector2f& p, int button, bool down) override;
  void drag(const Eigen::Vector2f& p, int button) override;
  void scroll(const Eigen::Vector2f& rel) override;

  void updown(double p) override;
  void arrow(const Eigen::Vector2f& p) override;
  void update() override;

  Eigen::Vector2f depth_range() const override;
  Eigen::Matrix4f view_matrix() const override;

private:
  Eigen::Vector3f pos;
  double yaw;
  double pitch;

  bool fovy_locked;
  double min_fovy;
  double max_fovy;
  double default_fovy;

  Eigen::Vector2i size;
  double fovy;
  double near_; // near, far are reserved words in MSVC!
  double far_;

  double mouse_sensitivity_yaw;
  double mouse_sensitivity_pitch;
  double translation_speed;

  bool left_button_down;
  bool right_button_down;
  bool middle_button_down;
  Eigen::Vector2f drag_last_pos;
};

}  // namespace  guik

#endif
