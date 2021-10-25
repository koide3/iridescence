#include <glk/io/ascii_io.hpp>

#include <fstream>
#include <glk/console_colors.hpp>

namespace glk {

template<typename T, int D>
bool save_ascii(const std::string& filename, const Eigen::Matrix<T, D, 1>* points, int num_points) {
  std::ofstream ofs(filename);
  if(!ofs) {
    std::cerr << console::bold_red << "error: failed to open " << filename << console::reset << std::endl;
    return false;
  }

  for(int i = 0; i < num_points; i++) {
    for(int j = 0; j < D; j++) {
      ofs << points[i][j] << " ";
    }
    ofs << std::endl;
  }

  return true;
}

template bool save_ascii(const std::string& filename, const Eigen::Matrix<float, 3, 1>* points, int num_points);
template bool save_ascii(const std::string& filename, const Eigen::Matrix<float, 4, 1>* points, int num_points);
template bool save_ascii(const std::string& filename, const Eigen::Matrix<double, 3, 1>* points, int num_points);
template bool save_ascii(const std::string& filename, const Eigen::Matrix<double, 4, 1>* points, int num_points);

}  // namespace glk