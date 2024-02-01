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

void HistogramPlotData::plot() const {
  if (ys.empty()) {
    ImPlot::PlotHistogram(label.c_str(), xs.data(), xs.size(), x_bins, 1.0, ImPlotRange(x_range_min, x_range_max), histogram_flags);
  } else {
    ImPlot::PlotHistogram2D(label.c_str(), xs.data(), ys.data(), xs.size(), x_bins, y_bins, ImPlotRect(x_range_min, x_range_max, y_range_min, y_range_max), histogram_flags);
  }
}

void LinePlotStyle::apply() const {
  ImPlot::SetNextLineStyle(ImVec4(col[0], col[1], col[2], col[3]), weight);
}

void ScatterPlotStyle::apply() const {
  ImPlot::SetNextMarkerStyle(marker, size, ImVec4(fill[0], fill[1], fill[2], fill[3]), weight, ImVec4(outline[0], outline[1], outline[2], outline[3]));
}

}  // namespace guik
