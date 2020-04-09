#include <glk/pointnormals_buffer.hpp>

namespace glk {

PointNormalsBuffer::PointNormalsBuffer(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud, double normal_length) {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lines(cloud->size() * 2);

  for(int i = 0; i < cloud->size(); i++) {
    lines[i * 2] = cloud->at(i).getVector3fMap();
    lines[i * 2 + 1] = cloud->at(i).getVector3fMap() + cloud->at(i).getNormalVector3fMap() * normal_length;
  }

  normal_lines.reset(new ThinLines(lines));
}

PointNormalsBuffer::~PointNormalsBuffer() {}

void PointNormalsBuffer::draw(glk::GLSLShader& shader) const {
  normal_lines->draw(shader);
}

}  // namespace glk