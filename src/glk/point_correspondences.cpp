#include <glk/point_correspondences.hpp>

namespace glk {

#ifdef GLK_USE_PCL

template <typename PointT>
PointCorrespondences::PointCorrespondences(
  const pcl::shared_ptr<const pcl::PointCloud<PointT>>& source_cloud,
  const pcl::shared_ptr<const pcl::PointCloud<PointT>>& target_cloud,
  const pcl::CorrespondencesConstPtr& correspondences) {
  std::vector<Eigen::Vector3f> vertices(correspondences->size() * 2);
  for (const auto& c : *correspondences) {
    vertices.push_back(source_cloud->at(c.index_query).getVector3fMap());
    vertices.push_back(target_cloud->at(c.index_match).getVector3fMap());
  }

  lines.reset(new glk::ThinLines(vertices));
}

template PointCorrespondences::PointCorrespondences(
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& source_cloud,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& target_cloud,
  const pcl::CorrespondencesConstPtr& correspondences);
template PointCorrespondences::PointCorrespondences(
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& source_cloud,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& target_cloud,
  const pcl::CorrespondencesConstPtr& correspondences);
template PointCorrespondences::PointCorrespondences(
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointNormal>>& source_cloud,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointNormal>>& target_cloud,
  const pcl::CorrespondencesConstPtr& correspondences);

#endif

PointCorrespondences::~PointCorrespondences() {}

void PointCorrespondences::draw(glk::GLSLShader& shader) const {
  lines->draw(shader);
}

}  // namespace glk