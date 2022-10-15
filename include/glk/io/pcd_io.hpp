#ifndef GLK_PCD_IO_HPP
#define GLK_PCD_IO_HPP

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace glk {

struct PCDData {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<float> intensities;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
};

std::shared_ptr<PCDData> load_pcd(const std::string& filename);

}  // namespace glk

#endif