#ifndef GUIK_ARCBALL_CAMERA_CONTROL_XY_HPP
#define GUIK_ARCBALL_CAMERA_CONTROL_XY_HPP

#include <guik/camera/camera_control.hpp>

namespace guik {

class ArcBallCameraControl : public CameraControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ArcBallCameraControl();
  ArcBallCameraControl(double distance, const Eigen::Quaternionf& quat = Eigen::Quaternionf::Identity());
  virtual ~ArcBallCameraControl() override;

  virtual void reset_center() override;
  virtual void lookat(const Eigen::Vector3f& pt) override;

  virtual void mouse(const Eigen::Vector2f& p, int button, bool down) override;
  virtual void drag(const Eigen::Vector2f& p, int button) override;
  virtual void scroll(const Eigen::Vector2f& rel) override;

  virtual void updown(double p) override;
  virtual void arrow(const Eigen::Vector2f& p) override;

  virtual Eigen::Vector2f depth_range() const override;

  virtual Eigen::Matrix4f view_matrix() const override;

  virtual std::string name() const override { return "ArcBallCameraControl"; }
  virtual void load(std::istream& ist) override;
  virtual void save(std::ostream& ost) const override;

protected:
  double distance;
  Eigen::Vector3f center;
  Eigen::Vector3f center_offset;

  Eigen::Vector3f delta_trans;

  Eigen::Quaternionf orientation;
  Eigen::Quaternionf delta_orientation;

  bool left_button_down;
  bool middle_button_down;
  Eigen::Vector2f mouse_down_pos;
  Eigen::Vector2f drag_last_pos;
};

}  // namespace guik

#endif