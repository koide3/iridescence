#ifndef GLK_CAMERA_CONTROL_HPP
#define GLK_CAMERA_CONTROL_HPP

#include <memory>
#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace guik {

class CameraControl {
public:
  virtual ~CameraControl() {}

  virtual void reset_center() {}

  virtual void lookat(const Eigen::Vector3f& pt) {}

  virtual void mouse(const Eigen::Vector2i& p, int button, bool down) = 0;
  virtual void drag(const Eigen::Vector2i& p, int button) = 0;
  virtual void scroll(const Eigen::Vector2f& rel) = 0;

  virtual void updown(int p) {}
  virtual void arrow(const Eigen::Vector2i& p) {}

  virtual Eigen::Vector2f depth_range() const {
    return Eigen::Vector2f(0.0f, 1.0f);
  }

  virtual Eigen::Matrix4f view_matrix() const = 0;
};

}  // namespace guik

#endif