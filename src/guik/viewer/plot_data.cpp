#include <guik/viewer/plot_data.hpp>

#include <implot.h>

namespace guik {

void LinePlotData::plot() const {
  ImPlot::PlotLine(label.c_str(), xs.data(), ys.data(), xs.size(), line_flags);
}

void ScatterPlotData::plot() const {
  ImPlot::PlotScatter(label.c_str(), xs.data(), ys.data(), xs.size(), scatter_flags);
}

void StairsPlotData::plot() const {
  ImPlot::PlotStairs(label.c_str(), xs.data(), ys.data(), xs.size(), stairs_flags);
}

}  // namespace guik
