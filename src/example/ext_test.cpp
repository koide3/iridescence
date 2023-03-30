#include <iostream>
#include <glk/type_conversion.hpp>
#include <glk/thin_lines.hpp>
#include <guik/viewer/light_viewer.hpp>

class TestClass {
public:
  template <template <class> class Allocator>
  TestClass(const std::vector<Eigen::Vector3f, Allocator<Eigen::Vector3f>>& vertices) {
    std::cout << "constructor1" << std::endl;
  }

  template <typename Scalar, int Dim, template <class> class Allocator>
  TestClass(const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator<Eigen::Matrix<Scalar, Dim, 1>>>& vertices) {
    std::cout << "constructor2" << std::endl;
  }

  // template <typename Scalar, int Dim, typename = std::enable_if_t<Dim != 3>>
  // TestClass(const Eigen::Matrix<Scalar, Dim, 1>* vertices, int num_vertices, bool line_strip = false);
};

int main(int argc, char** argv) {
  std::vector<Eigen::Vector4d> vertices_4d;
  for (double t = 0.0; t < 3.0; t += 0.1) {
    vertices_4d.emplace_back(std::cos(t), std::sin(t), 0.0, 1.0);
  }

  auto viewer = guik::LightViewer::instance();
  // viewer->update_thin_lines("test", vertices_3f.data(), 5, false, guik::Rainbow());
  // viewer->update_thin_lines("test", vertices_4d, indices, false, guik::Rainbow());

  for (double x = 0.0; x < 10.0; x += 2.0) {
    Eigen::Vector4d trans(x, 0.0, 0.0, 1.0);
    viewer->update_coord("coord_" + std::to_string(x), guik::VertexColor().rotate(Eigen::Quaterniond::Identity()));
  }

  viewer->spin();

  /*
  std::vector<Eigen::Vector4d> values(128, Eigen::Vector4d(1, 2, 3, 4));

  auto converted = glk::convert_vector<double, 4>(values);
  auto converted2 = glk::convert_vector<float, 3>(values);
  auto converted3 = glk::convert_vector<double, 4>(std::vector<Eigen::Vector4d>(values));

  for (int i = 0; i < 3; i++) {
    std::cout << converted[i].transpose() << " : " << converted2[i].transpose() << std::endl;
  }
  */

  return 0;
}