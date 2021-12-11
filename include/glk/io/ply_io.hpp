#ifndef GLK_PLY_IO_HPP
#define GLK_PLY_IO_HPP

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace glk {

struct PLYMetaData {
  enum class PropertyType { CHAR, UCHAR, SHORT, USHORT, INT, UINT, FLOAT, DOUBLE };

  std::string format;

  int num_vertices;
  int num_faces;
  std::vector<std::pair<std::string, PropertyType>> vertex_properties;
  std::vector<PropertyType> face_properties;
};

struct PLYData {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<float> intensities;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
  std::vector<int> indices;
};

std::shared_ptr<PLYData> load_ply(const std::string& filename);

bool save_ply_ascii(const std::string& filename, const PLYData& ply);
bool save_ply_binary(const std::string& filename, const PLYData& ply);

template<typename T, int D>
bool save_ply_binary(const std::string& filename, const Eigen::Matrix<T, D, 1>* points, int num_points);

}  // namespace glk

#endif