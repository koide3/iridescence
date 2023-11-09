#ifndef GUIK_STATIC_CAMERA_CONTROL_HPP
#define GUIK_STATIC_CAMERA_CONTROL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <guik/camera/camera_control.hpp>

namespace guik {

class StaticCameraControl : public CameraControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StaticCameraControl();
  StaticCameraControl(const Eigen::Isometry3f& T_world_camera, const Eigen::Vector2f& depth = Eigen::Vector2f(1e-3, 1e3));
  StaticCameraControl(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up, const Eigen::Vector2f& depth = Eigen::Vector2f(1e-3, 1e3));
  virtual ~StaticCameraControl() override;

  virtual Eigen::Vector2f depth_range() const override;
  virtual Eigen::Matrix4f view_matrix() const override;

  virtual std::string name() const override { return "StaticCameraControl"; }
  virtual void load(std::istream& ist) override;
  virtual void save(std::ostream& ost) const override;

protected:
  Eigen::Vector2f depth;
  Eigen::Matrix4f matrix;
};

}  // namespace guik

#endif