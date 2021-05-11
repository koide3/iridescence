#ifndef GUIK_ARCBALL_CAMERA_CONTROL_XY_HPP
#define GUIK_ARCBALL_CAMERA_CONTROL_XY_HPP

#include <guik/camera/camera_control.hpp>

namespace guik {

class ArcBallCameraControl : public CameraControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ArcBallCameraControl();
  ArcBallCameraControl(double distance);
  virtual ~ArcBallCameraControl() override;

  virtual void reset_center() override;
  virtual void lookat(const Eigen::Vector3f& pt) override;

  virtual void mouse(const Eigen::Vector2i& p, int button, bool down) override;
  virtual void drag(const Eigen::Vector2i& p, int button) override;
  virtual void scroll(const Eigen::Vector2f& rel) override;

  virtual void updown(int p) override;
  virtual void arrow(const Eigen::Vector2i& p) override;

  virtual Eigen::Vector2f depth_range() const override;

  virtual Eigen::Matrix4f view_matrix() const override;

protected:
  double distance;
  Eigen::Vector3f center;
  Eigen::Vector3f center_offset;

  Eigen::Quaternionf orientation;
  Eigen::Quaternionf delta_orientation;

  bool left_button_down;
  bool middle_button_down;
  Eigen::Vector2i drag_last_pos;
};

}  // namespace guik

#endif