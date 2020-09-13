#ifndef GLK_POINTCLOUD_BUFFER_PCL_HPP
#define GLK_POINTCLOUD_BUFFER_PCL_HPP

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <glk/pointcloud_buffer.hpp>

namespace glk {

template<typename PointT>
std::shared_ptr<PointCloudBuffer> create_point_cloud_buffer(const pcl::PointCloud<PointT>& cloud) {
  return std::make_shared<PointCloudBuffer>(&cloud.at(0).x, sizeof(PointT), cloud.size());
}

// template PointCloudBuffer::PointCloudBuffer(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& cloud);

}  // namespace glk

#endif