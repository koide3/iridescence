#ifndef GUIK_PLOT_SETTING_HPP
#define GUIK_PLOT_SETTING_HPP

#include <memory>
#include <vector>
#include <string>

namespace guik {

struct PlotSetting {
public:
  PlotSetting()
  : width(512),
    height(256),
    plot_flags(0),
    x_flags(0),
    y_flags(0),
    legend_loc(1 + 4),
    legend_flags(0),
    order(0),
    axis_link_id(-1),
    linked_axes(0),
    set_axes_to_fit(false) {}
  ~PlotSetting() {}

public:
  int width;
  int height;

  std::string x_label;
  std::string y_label;

  int plot_flags;
  int x_flags;
  int y_flags;

  int legend_loc;
  int legend_flags;

  int order;

  int axis_link_id;  // -1 for invalid
  int linked_axes;   // 1 << ImAxis_X1 | 1 << ImAxis_X2 | ...

  bool set_axes_to_fit;
};
}  // namespace guik

#endif