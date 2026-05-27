#ifndef GLK_PRIMITIVES_FRUSTUM_HPP
#define GLK_PRIMITIVES_FRUSTUM_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

class Frustum {
public:
  Frustum(float w, float h, float z, bool close_front) {
    vertices.push_back(Eigen::Vector3f(w / 2, h / 2, z));    // v0
    vertices.push_back(Eigen::Vector3f(-w / 2, h / 2, z));   // v1
    vertices.push_back(Eigen::Vector3f(-w / 2, -h / 2, z));  // v2
    vertices.push_back(Eigen::Vector3f(w / 2, -h / 2, z));   // v3
    vertices.push_back(Eigen::Vector3f(0, 0, 0));            // v4

    normals.push_back(vertices[0].normalized());  // n1
    normals.push_back(vertices[1].normalized());  // n2
    normals.push_back(vertices[2].normalized());  // n3
    normals.push_back(vertices[3].normalized());  // n4
    normals.push_back(Eigen::Vector3f(0, 0, 0));  // n5

    indices = {0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0};

    if (close_front) {
      std::array<int, 6> front_indices = {0, 1, 2, 0, 2, 3};
      indices.insert(indices.end(), front_indices.begin(), front_indices.end());
    }
  }

  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector3f> normals;
  std::vector<int> indices;
};
}  // namespace glk

#endif