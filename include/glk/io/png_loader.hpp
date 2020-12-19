#ifndef GLK_PNG_LOADER_HPP
#define GLK_PNG_LOADER_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

class PNGLoader {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PNGLoader();
  ~PNGLoader();

  bool load(const std::string& filename);

  Eigen::Vector2i size() const {
    return Eigen::Vector2i(width, height);
  }

private:
  bool load(FILE* fp);

public:
  int width;
  int height;
  std::vector<unsigned char> bytes;
};

}  // namespace glk

#endif