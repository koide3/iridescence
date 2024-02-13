#ifndef GUIK_TOPDOWN_CAMERA_CONTROL_HPP
#define GUIK_TOPDOWN_CAMERA_CONTROL_HPP

#include <guik/camera/camera_control.hpp>

namespace guik {

class TopDownCameraControl : public CameraControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TopDownCameraControl();
  TopDownCameraControl(double distance, double theta);
  virtual ~TopDownCameraControl() override;

  virtual void reset_center() override;
  virtual void lookat(const Eigen::Vector3f& pt) override;
  virtual void mouse(const Eigen::Vector2f& p, int button, bool down) override;
  virtual void drag(const Eigen::Vector2f& p, int button) override;
  virtual void scroll(const Eigen::Vector2f& rel) override;
  virtual Eigen::Vector2f depth_range() const override;

  virtual Eigen::Matrix4f view_matrix() const override;

  virtual std::string name() const override { return "TopDownCameraControl"; }
  virtual void load(std::istream& ist) override;
  virtual void save(std::ostream& ost) const override;

protected:
  Eigen::Vector3f center_offset;
  Eigen::Vector3f center;
  double distance;

  Eigen::Vector2f drag_last_pos;

  bool left_button_down;
  bool middle_button_down;
  double theta;
};

}  // namespace guik

#endif