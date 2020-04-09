#ifndef GLK_POINT_NORMALS_BUFFER_HPP
#define GLK_POINT_NORMALS_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/drawble.hpp>
#include <glk/thin_lines.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

class PointNormalsBuffer : public glk::Drawable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PointNormalsBuffer>;

  PointNormalsBuffer(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud, double normal_length=0.1);
  virtual ~PointNormalsBuffer() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::ThinLines> normal_lines;
};

}  // namespace glk

#endif