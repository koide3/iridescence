#ifndef GUIK_PLOT_DATA_HPP
#define GUIK_PLOT_DATA_HPP

#include <memory>
#include <vector>
#include <string>

namespace guik {

struct PlotData {
public:
  using Ptr = std::shared_ptr<PlotData>;
  using ConstPtr = std::shared_ptr<const PlotData>;

  PlotData(const std::string& label) : label(label) {}
  virtual ~PlotData() {}

  virtual void plot() const = 0;

public:
  std::string label;
};

struct LinePlotData : public PlotData {
public:
  LinePlotData(const std::string& label) : PlotData(label), line_flags(0) {}
  ~LinePlotData() {}

  virtual void plot() const override;

public:
  int line_flags;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct ScatterPlotData : public PlotData {
public:
  ScatterPlotData(const std::string& label) : PlotData(label), scatter_flags(0) {}
  ~ScatterPlotData() {}

  virtual void plot() const override;

public:
  int scatter_flags;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct StairsPlotData : public PlotData {
public:
  StairsPlotData(const std::string& label) : PlotData(label), stairs_flags(0) {}
  ~StairsPlotData() {}

  virtual void plot() const override;

public:
  int stairs_flags;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct HistogramPlotData : public PlotData {
public:
  HistogramPlotData(const std::string& label)
  : PlotData(label),
    histogram_flags(0),
    x_bins(-2),
    y_bins(-2),
    x_range_min(0.0),
    x_range_max(0.0),
    y_range_min(0.0),
    y_range_max(0.0) {}
  ~HistogramPlotData() {}

  virtual void plot() const override;

public:
  int histogram_flags;
  int x_bins;
  int y_bins;
  double x_range_min;
  double x_range_max;
  double y_range_min;
  double y_range_max;
  std::vector<double> xs;
  std::vector<double> ys;
};

}  // namespace guik

#endif