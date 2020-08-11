#include <glk/pointcloud_buffer.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

PointCloudBuffer::PointCloudBuffer(const std::string& cloud_filename) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if(pcl::io::loadPCDFile(cloud_filename, *cloud)) {
    std::cerr << "error: failed to load " << cloud_filename << std::endl;
    num_points = 0;
    return;
  }

  stride = sizeof(pcl::PointXYZI);
  num_points = cloud->size();

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->points.data(), GL_STATIC_DRAW);
}

PointCloudBuffer::PointCloudBuffer(const float* data, int stride, int num_points) {
  this->stride = stride;
  this->num_points = num_points;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, stride * num_points, data, GL_STATIC_DRAW);
}

template<typename PointT>
PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<const pcl::PointCloud<PointT>>& cloud) {
  stride = sizeof(PointT);
  num_points = cloud->size();

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(PointT), cloud->points.data(), GL_STATIC_DRAW);
}

template<typename PointT>
PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<pcl::PointCloud<PointT>>& cloud) {
  stride = sizeof(PointT);
  num_points = cloud->size();

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(PointT), cloud->points.data(), GL_STATIC_DRAW);
}

template PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud);
template PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud);
template PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>>& cloud);
template PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& cloud);
template PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& cloud);
template PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<const pcl::PointCloud<pcl::PointNormal>>& cloud);

PointCloudBuffer::~PointCloudBuffer() {
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void PointCloudBuffer::draw(glk::GLSLShader& shader) const {
  if(num_points == 0) {
    return;
  }

  GLint position_loc = shader.attrib("vert_position");

  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);

  glDrawArrays(GL_POINTS, 0, num_points);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(0);
}

}  // namespace glk
