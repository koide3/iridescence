#include <guik/camera/static_camera_control.hpp>

namespace guik {

StaticCameraControl::StaticCameraControl() : depth(1e-3, 1e3), matrix(Eigen::Matrix4f::Identity()) {}

StaticCameraControl::StaticCameraControl(const Eigen::Isometry3f& T_world_camera, const Eigen::Vector2f& depth)
: depth(depth),
  matrix((T_world_camera * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())).inverse().matrix()) {}

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