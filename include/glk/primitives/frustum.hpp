#ifndef GLK_PRIMITIVES_FRUSTUM_HPP
#define GLK_PRIMITIVES_FRUSTUM_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

class Frustum {
public:
  Frustum(float w, float h, float z) {
    vertices.push_back(Eigen::Vector3f(w / 2, h / 2, z));
    vertices.push_back(Eigen::Vector3f(-w / 2, h / 2, z));
    vertices.push_back(Eigen::Vector3f(-w / 2, -h / 2, z));
    vertices.push_back(Eigen::Vector3f(w / 2, -h / 2, z));
    vertices.push_back(Eigen::Vector3f(0, 0, 0));

    normals.push_back(Eigen::Vector3f(0, 0, 1));
    normals.push_back(Eigen::Vector3f(0, 0, 1));
    normals.push_back(Eigen::Vector3f(0, 0, 1));
    normals.push_back(Eigen::Vector3f(0, 0, 1));

    for(int i = 0; i < 4; i++) {
      indices.push_back(i);
      indices.push_back((i + 1) % 4);

      indices.push_back(i);
      indices.push_back(4);
    }

    indices.push_back(0);
    indices.push_back(0);
  }

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<int> indices;
};
}  // namespace glk

#endif