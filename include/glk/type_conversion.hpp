#ifndef GLK_TYPE_CONVERSION_HPP
#define GLK_TYPE_CONVERSION_HPP

#include <vector>
#include <type_traits>
#include <Eigen/Core>

namespace glk {

template <typename Dst, typename Src>
std::vector<Dst> convert_scalars(const Src* data, int num_data) {
  std::vector<Dst> converted(num_data);
  std::copy(data, data + num_data, converted.begin());
  return converted;
}

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
std::enable_if_t<
  std::is_same<Dst_Scalar, Src_Scalar>::value,  //
  std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>>>
convert_to_vector(const Eigen::Matrix<Src_Scalar, Src_Rows, Src_Cols>* points, int num_points) {
  if (points == nullptr) {
    return std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>>(num_points);
  }

  if constexpr (Dst_Rows < Src_Rows && Dst_Cols == 1 && Src_Cols == 1) {
    std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>> converted(num_points);
    Eigen::Map<Eigen::Matrix<Dst_Scalar, Dst_Rows, -1>> dst_map(converted[0].data(), Dst_Rows, num_points);
    Eigen::Map<const Eigen::Matrix<Src_Scalar, Src_Rows, -1>> src_map(points[0].data(), Src_Rows, num_points);
    dst_map = src_map.template topRows<Dst_Rows>();
    return converted;

  } else {
    constexpr int Rows = std::min(Dst_Rows, Src_Rows);
    constexpr int Cols = std::min(Dst_Cols, Src_Cols);

    std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>> converted(num_points);
    for (int i = 0; i < num_points; i++) {
      converted[i].template block<Rows, Cols>(0, 0) = points[i].template cast<Dst_Scalar>().template block<Rows, Cols>(0, 0);
    }

    return converted;
  }
}

template <typename Dst_Scalar, int Dst_Rows, int Dst_Cols, typename Src_Scalar, int Src_Rows, int Src_Cols, template <class> class Allocator = std::allocator>
std::enable_if_t<
  !std::is_same<Dst_Scalar, Src_Scalar>::value,  //
  std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>>>
convert_to_vector(const Eigen::Matrix<Src_Scalar, Src_Rows, Src_Cols>* points, int num_points) {
  if (points == nullptr) {
    return std::vector<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Dst_Rows, Dst_Cols>>>(num_points);
  }

  std::vector<Eigen::Matrix<Dst_Scalar, Src_Rows, Src_Cols>, Allocator<Eigen::Matrix<Dst_Scalar, Src_Rows, Src_Cols>>> converted(num_points);
  Eigen::Map<Eigen::Matrix<Dst_Scalar, -1, 1>> dst_map(converted[0].data(), converted.size() * Src_Rows * Src_Cols);
  Eigen::Map<const Eigen::Matrix<Src_Scalar, -1, 1>> src_map(points[0].data(), num_points * Src_Rows * Src_Cols);
  dst_map = src_map.template cast<Dst_Scalar>();

  if constexpr (Dst_Rows == Src_Rows && Dst_Cols == Src_Cols) {
    return converted;
  } else {
    return convert_to_vector<Dst_Scalar, Dst_Rows, Dst_Cols>(converted.data(), num_points);
  }
}

}  // namespace glk

#endif