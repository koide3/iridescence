#ifndef GLK_COLORMAP_HPP
#define GLK_COLORMAP_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

enum class COLORMAP {
  TURBO = 0,
  JET,
  CIVIDIS,
  OCEAN,
  SPRING,
  SUMMER,
  AUTUMN,
  WINTER,
  GREAN_YELLOW,
  BLUE_RED,
  PUBUGN,
  TURBID,
  PASTEL,
  HELIX,
  PHASE,
  VEGETATION,
  CURL,
  COOL_WARM,
  NUM_COLORMAPS
};

Eigen::Vector4i colormap(COLORMAP type, int x);
Eigen::Vector4f colormapf(COLORMAP type, float x);
Eigen::Vector4i colormap_categorical(COLORMAP type, int x, int num_categories);
Eigen::Vector4f colormap_categoricalf(COLORMAP type, int x, int num_categories);

std::vector<const char*> colormap_names();
std::array<std::array<unsigned char, 3>, 256> colormap_table(COLORMAP type);

}  // namespace glk

#endif