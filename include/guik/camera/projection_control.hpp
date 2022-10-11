#ifndef GUIK_PROJECTION_CONTROL_HPP
#define GUIK_PROJECTION_CONTROL_HPP

#include <Eigen/Core>

namespace guik {

class ProjectionControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ProjectionControl() {}
  virtual ~ProjectionControl() {}

  virtual void set_size(const Eigen::Vector2i& size) = 0;

  virtual void set_depth_range(const Eigen::Vector2f& range) = 0;

  virtual Eigen::Matrix4f projection_matrix() const = 0;

  virtual void draw_ui(){};

  // io
  virtual std::string name() const { return "NONE"; }
  virtual void load(std::istream& ist){};
  virtual void save(std::ostream& ost) const {};
};

std::istream& operator>>(std::istream& ist, ProjectionControl& cam);
std::ostream& operator<<(std::ostream& ost, const ProjectionControl& cam);

}  // namespace guik

#endif