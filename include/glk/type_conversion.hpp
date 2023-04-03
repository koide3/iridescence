#ifndef GLK_TYPE_CONVERSION_HPP
#define GLK_TYPE_CONVERSION_HPP

#include <vector>
#include <type_traits>
#include <Eigen/Core>

namespace glk {

template <typename Dst_Scalar, int Dst_Dim, typename Src_Scalar, int Src_Dim, template <class> class Allocator>
std::enable_if_t<
  std::is_same<Dst_Scalar, Src_Scalar>::value && Dst_Dim == Src_Dim,
  std::vector<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>>>>
convert_vector(std::vector<Eigen::Matrix<Src_Scalar, Src_Dim, 1>, Allocator<Eigen::Matrix<Src_Scalar, Src_Dim, 1>>>&& data) {
  return std::move(data);
}

template <typename Dst_Scalar, int Dst_Dim, typename Src_Scalar, int Src_Dim, template <class> class Allocator>
std::enable_if_t<
  std::is_same<Dst_Scalar, Src_Scalar>::value && Dst_Dim == Src_Dim,
  std::vector<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>>>>
convert_vector(const std::vector<Eigen::Matrix<Src_Scalar, Src_Dim, 1>, Allocator<Eigen::Matrix<Src_Scalar, Src_Dim, 1>>>& data) {
  return data;
}

template <typename Dst_Scalar, int Dst_Dim, typename Src_Scalar, int Src_Dim, template <class> class Allocator>
std::enable_if_t<
  !std::is_same<Dst_Scalar, Src_Scalar>::value || Dst_Dim != Src_Dim,
  std::vector<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>>>>
convert_vector(const std::vector<Eigen::Matrix<Src_Scalar, Src_Dim, 1>, Allocator<Eigen::Matrix<Src_Scalar, Src_Dim, 1>>>& data) {
  std::vector<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Dim, 1>>> converted(data.size());
  std::transform(data.begin(), data.end(), converted.begin(), [](const Eigen::Matrix<Src_Scalar, Src_Dim, 1>& x) {
    return x.template cast<Dst_Scalar>().template head<Dst_Dim>();
  });
  return converted;
}

template <typename Dst_Scalar, int Dst_Rows, int Dst_Cols, typename Src_Scalar, int Src_Rows, int Src_Cols, template <class> class Allocator = std::allocator>
std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>> convert_to_vector(
  const Eigen::Matrix<Src_Scalar, Src_Rows, Src_Cols>* points,
  int num_points) {
  if (points == nullptr) {
    return std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>>(num_points);
  }

  constexpr int Rows = std::min(Dst_Rows, Src_Rows);
  constexpr int Cols = std::min(Dst_Cols, Src_Cols);

  std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>> converted(num_points);
  for (int i = 0; i < num_points; i++) {
    converted[i].template block<Rows, Cols>(0, 0) = points[i].template cast<Dst_Scalar>().template block<Rows, Cols>(0, 0);
  }
  return converted;
}

}  // namespace glk

#endif