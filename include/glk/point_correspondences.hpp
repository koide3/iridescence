#ifndef GLK_POINT_CORRESPONDENCES_BUFFER_HPP
#define GLK_POINT_CORRESPONDENCES_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/drawble.hpp>
#include <glk/thin_lines.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>

namespace glk {

class PointCorrespondences : public glk::Drawable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PointCorrespondences>;

  template<typename PointT>
  PointCorrespondences(const boost::shared_ptr<const pcl::PointCloud<PointT>>& source_cloud, const boost::shared_ptr<const pcl::PointCloud<PointT>>& target_cloud, const pcl::CorrespondencesConstPtr& correspondences);
  virtual ~PointCorrespondences() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::ThinLines> lines;
};

}  // namespace glk

#endif