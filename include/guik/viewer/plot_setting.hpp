#ifndef GUIK_PLOT_SETTING_HPP
#define GUIK_PLOT_SETTING_HPP

#include <memory>
#include <vector>
#include <string>

namespace guik {

struct PlotSetting {
public:
  PlotSetting() : width(512), height(256), plot_flags(0), x_flags(0), y_flags(0), order(0), set_axes_to_fit(false) {}
  ~PlotSetting() {}

public:
  int width;
  int height;

  std::string x_label;
  std::string y_label;

  int plot_flags;
  int x_flags;
  int y_flags;

  int order;

  bool set_axes_to_fit;
};
}  // namespace guik

#endif