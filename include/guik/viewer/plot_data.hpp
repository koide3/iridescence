#ifndef GUIK_PLOT_DATA_HPP
#define GUIK_PLOT_DATA_HPP

#include <memory>
#include <vector>
#include <string>

namespace guik  {

struct PlotData {
public:
  using Ptr = std::shared_ptr<PlotData>;
  using ConstPtr = std::shared_ptr<const PlotData>;

  PlotData() : x_flags(0), y_flags(0) {}
  ~PlotData() {}

public:
  std::vector<double> xs;
  std::vector<double> ys;

  std::string label;
  std::string x_label;
  std::string y_label;

  int x_flags;
  int y_flags;
};
}

#endif