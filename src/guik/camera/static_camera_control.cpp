#include <guik/camera/static_camera_control.hpp>

#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace guik {

StaticCameraControl::StaticCameraControl() : depth(1e-3, 1e3), matrix(Eigen::Matrix4f::Identity()) {}

StaticCameraControl::StaticCameraControl(const Eigen::Isometry3f& T_world_camera, const Eigen::Vector2f& depth)
: depth(depth),
  matrix((T_world_camera * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())).inverse().matrix()) {}

StaticCameraControl::StaticCameraControl(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up, const Eigen::Vector2f& depth) : depth(depth) {
  const glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center[0], center[1], center[2]), glm::vec3(up[0], up[1], up[2]));
  this->matrix = Eigen::Map<const Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
}

StaticCameraControl::~StaticCameraControl() {}

Eigen::Vector2f StaticCameraControl::depth_range() const {
  return depth;
}

Eigen::Matrix4f StaticCameraControl::view_matrix() const {
  return matrix;
}

void StaticCameraControl::load(std::istream& ist) {
  std::string token;
  ist >> token >> depth[0] >> depth[1];

  ist >> token;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ist >> matrix(i, j);
    }
  }
}

void StaticCameraControl::save(std::ostream& ost) const {
  ost << "depth: " << depth.transpose() << std::endl;
  ost << "matrix: " << std::endl << matrix << std::endl;
}

}  // namespace guik