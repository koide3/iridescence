#ifndef GLK_POINT_CORRESPONDENCES_BUFFER_HPP
#define GLK_POINT_CORRESPONDENCES_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/drawable.hpp>
#include <glk/thin_lines.hpp>

#ifdef GLK_USE_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#endif

namespace glk {

class PointCorrespondences : public glk::Drawable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PointCorrespondences>;

#ifdef GLK_USE_PCL
  template <typename PointT>
  PointCorrespondences(
    const pcl::shared_ptr<const pcl::PointCloud<PointT>>& source_cloud,
    const pcl::shared_ptr<const pcl::PointCloud<PointT>>& target_cloud,
    const pcl::CorrespondencesConstPtr& correspondences);
#endif

  virtual ~PointCorrespondences() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::unique_ptr<glk::ThinLines> lines;
};

}  // namespace glk

#endif