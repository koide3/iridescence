#ifndef GUIK_ORBIT_CAMERA_CONTROL_XY_HPP
#define GUIK_ORBIT_CAMERA_CONTROL_XY_HPP

#include <guik/camera/camera_control.hpp>

namespace guik {

/// @brief Orbit camera control (Up vector is locked to Z axis)
///        Left button: rotate (pan/tilt)
///        Middle or Right button: XY translate
///        Middle &  Right button: Z translate
///        Scroll wheel: zoom in/out
class OrbitCameraControlXY : public CameraControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OrbitCameraControlXY();
  OrbitCameraControlXY(double theta, double phi, double distance);
  virtual ~OrbitCameraControlXY() override;

  virtual void reset_center() override;
  virtual void lookat(const Eigen::Vector3f& pt) override;

  virtual void mouse(const Eigen::Vector2f& p, int button, bool down) override;
  virtual void drag(const Eigen::Vector2f& p, int button) override;
  virtual void scroll(const Eigen::Vector2f& rel) override;

  virtual void updown(double p) override;
  virtual void arrow(const Eigen::Vector2f& p) override;

  virtual Eigen::Vector2f depth_range() const override;

  virtual Eigen::Quaternionf rotation() const;
  virtual Eigen::Matrix4f view_matrix() const override;

  virtual std::string name() const override { return "OrbitCameraControlXY"; }
  virtual void load(std::istream& ist) override;
  virtual void save(std::ostream& ost) const override;

protected:
  Eigen::Vector3f center_offset;
  Eigen::Vector3f center;
  double distance;

  Eigen::Vector2f drag_last_pos;

  bool left_button_down;
  bool right_button_down;
  double theta;
  double phi;

  bool middle_button_down;
};

}  // namespace guik

#endif