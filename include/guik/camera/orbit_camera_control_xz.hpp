#ifndef GUIK_ORBIT_CAMERA_CONTROL_XZ_HPP
#define GUIK_ORBIT_CAMERA_CONTROL_XZ_HPP

#include <guik/camera/orbit_camera_control_xy.hpp>

namespace guik {

class OrbitCameraControlXZ : public OrbitCameraControlXY {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OrbitCameraControlXZ();
  OrbitCameraControlXZ(double distance, double theta, double phi);

  virtual ~OrbitCameraControlXZ() override;

  virtual void drag(const Eigen::Vector2f& p, int button) override;

  virtual Eigen::Quaternionf rotation() const override;
  virtual Eigen::Matrix4f view_matrix() const override;

  virtual std::string name() const override { return "OrbitCameraControlXZ"; }
  virtual void load(std::istream& ist) override;
  virtual void save(std::ostream& ost) const override;};

}  // namespace guik

#endif