#ifndef GLK_IO_ASCII_IO
#define GLK_IO_ASCII_IO

#include <string>
#include <Eigen/Core>

namespace glk {

template<typename T, int D>
bool save_ascii(const std::string& filename, const Eigen::Matrix<T, D, 1>* points, int num_points);

}

#endif