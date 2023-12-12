#include <implot.h>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();

  std::vector<double> xs;
  std::vector<double> ys_sin;
  for (double t = 0.0; t < 2.0 * M_PI; t += 0.1) {
    xs.emplace_back(t);
    ys_sin.emplace_back(std::sin(t));
  }

  // Basic plotting
  viewer->setup_plot("curves_y", 1024, 256);
  viewer->update_plot_line("curves_y", "sin", ys_sin);  // When only Y values are given, X values become index IDs
  viewer->update_plot_stairs("curves_y", "sin_stairs", ys_sin);

  viewer->setup_plot("curves_xy", 1024, 256);
  viewer->update_plot_line("curves_xy", "sin", xs, ys_sin);
  viewer->update_plot_stairs("curves_xy", "sin_stairs", xs, ys_sin);

  std::vector<double> xs_circle;
  std::vector<double> ys_circle;
  for (double t = 0.0; t < 2.0 * M_PI; t += 0.1) {
    xs_circle.emplace_back(std::cos(t));
    ys_circle.emplace_back(std::sin(t));
  }

  // If a plot name contains "/", the string before the slash is recognized as a group name.
  // Plots with the same group name are displayed in the same tab.
  viewer->setup_plot("group02/circle", 1024, 1024, ImPlotFlags_Equal);
  viewer->update_plot_line("group02/circle", "circle", xs_circle, ys_circle, ImPlotLineFlags_Loop);

  viewer->spin();
  return 0;
}