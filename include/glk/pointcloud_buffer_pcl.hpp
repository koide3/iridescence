#ifndef GLK_POINTCLOUD_BUFFER_PCL_HPP
#define GLK_POINTCLOUD_BUFFER_PCL_HPP

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <glk/pointcloud_buffer.hpp>

namespace glk {

template <typename PointT>
std::shared_ptr<PointCloudBuffer> create_point_cloud_buffer(const pcl::PointCloud<PointT>& cloud) {
  std::vector<Eigen::Vector3f> points(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    points[i] = cloud[i].getVector3fMap();
  }
  return std::make_shared<PointCloudBuffer>(points[0].data(), sizeof(Eigen::Vector3f), cloud.size());
}

template <>
inline std::shared_ptr<PointCloudBuffer> create_point_cloud_buffer(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  return std::make_shared<PointCloudBuffer>(&cloud.at(0).x, sizeof(pcl::PointXYZ), cloud.size());
}

template <>
inline std::shared_ptr<PointCloudBuffer> create_point_cloud_buffer(const pcl::PointCloud<pcl::PointNormal>& cloud) {
  std::vector<Eigen::Vector3f> points(cloud.size());
  std::vector<Eigen::Vector3f> normals(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    points[i] = cloud[i].getVector3fMap();
    normals[i] = cloud[i].getNormalVector3fMap();
  }

  auto cloud_buffer = std::make_shared<PointCloudBuffer>(points[0].data(), sizeof(Eigen::Vector3f), cloud.size());
  cloud_buffer->add_normals(normals[0].data(), sizeof(Eigen::Vector3f), normals.size());

  return cloud_buffer;
}

template <typename PointT>
std::shared_ptr<PointCloudBuffer> create_colored_point_cloud_buffer(const pcl::PointCloud<PointT>& cloud) {
  std::vector<Eigen::Vector3f> points(cloud.size());
  std::vector<Eigen::Vector4f> colors(cloud.size());

  for (int i = 0; i < cloud.size(); i++) {
    points[i] = cloud[i].getVector3fMap();
    colors[i] = Eigen::Vector4f(cloud[i].r, cloud[i].g, cloud[i].b, cloud[i].a) / 255.0f;
  }

  auto cloud_buffer = std::make_shared<PointCloudBuffer>(points[0].data(), sizeof(Eigen::Vector3f), points.size());
  cloud_buffer->add_color(colors[0].data(), sizeof(Eigen::Vector4f), colors.size());

  return cloud_buffer;
}

}  // namespace glk

#endif