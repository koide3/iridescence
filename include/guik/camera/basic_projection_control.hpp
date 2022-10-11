#ifndef GUIK_BASIC_PROJECTION_CONTROL_HPP
#define GUIK_BASIC_PROJECTION_CONTROL_HPP

#include <guik/camera/projection_control.hpp>

namespace guik {

class BasicProjectionControl : public ProjectionControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicProjectionControl(const Eigen::Vector2i& size);
  ~BasicProjectionControl();

  virtual void set_size(const Eigen::Vector2i& size) override { this->size = size; }

  virtual void set_depth_range(const Eigen::Vector2f& range) override;

  virtual Eigen::Matrix4f projection_matrix() const override;

  virtual void draw_ui() override;

  // io
  virtual std::string name() const override;
  virtual void load(std::istream& ist) override;
  virtual void save(std::ostream& ost) const override;

private:
  Eigen::Vector2i size;

  int projection_mode;

  float fovy;
  float width;
  float near;
  float far;
};

}  // namespace guik

#endif