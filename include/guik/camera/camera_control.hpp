#ifndef GLK_CAMERA_CONTROL_HPP
#define GLK_CAMERA_CONTROL_HPP

#include <memory>
#include <iostream>
#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace guik {

class CameraControl {
public:
  virtual ~CameraControl() {}

  virtual void reset_center() {}

  virtual void lookat(const Eigen::Vector3f& pt) {}

  virtual void mouse(const Eigen::Vector2f& p, int button, bool down) {}
  virtual void drag(const Eigen::Vector2f& p, int button) {}
  virtual void scroll(const Eigen::Vector2f& rel) {}

  virtual void update() {}
  virtual void updown(double p) {}
  virtual void arrow(const Eigen::Vector2f& p) {}

  virtual Eigen::Vector2f depth_range() const {
    return Eigen::Vector2f(0.0f, 1.0f);
  }

  virtual Eigen::Matrix4f view_matrix() const = 0;

  // io
  virtual std::string name() const { return "CameraControl"; }
  virtual void load(std::istream& ist) {}
  virtual void save(std::ostream& ost) const {}
};

std::istream& operator>> (std::istream& ist, CameraControl& cam);
std::ostream& operator<< (std::ostream& ost, const CameraControl& cam);

}  // namespace guik

#endif