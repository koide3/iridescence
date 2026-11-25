#include <deque>
#include <random>
#include <implot.h>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();

  std::vector<double> xs;
  std::vector<double> ys_sin;
  std::vector<double> ys_cos;
  for (double t = 0.0; t < 2.0 * M_PI; t += 0.1) {
    xs.emplace_back(t);
    ys_sin.emplace_back(std::sin(t));
    ys_cos.emplace_back(std::cos(t));
  }

  // ---- Basic plotting ----
  // If a plot name contains "/", the string before the slash is recognized as a group name.
  // Plots with the same group name are displayed in the same tab.
  viewer->setup_plot("basic/curves_y", 1024, 256);
  viewer->update_plot_line("basic/curves_y", "sin", ys_sin);  // When only Y values are given, X values become index IDs
  viewer->update_plot_stairs("basic/curves_y", "sin_stairs", ys_sin);

  viewer->setup_plot("basic/curves_xy", 1024, 256);
  viewer->update_plot_line("basic/curves_xy", "sin", xs, ys_sin);
  viewer->update_plot_stairs("basic/curves_xy", "sin_stairs", xs, ys_sin);

  // Equal axis scales and a closed loop
  std::vector<double> xs_circle;
  std::vector<double> ys_circle;
  for (double t = 0.0; t < 2.0 * M_PI; t += 0.1) {
    xs_circle.emplace_back(std::cos(t));
    ys_circle.emplace_back(std::sin(t));
  }
  viewer->setup_plot("basic/circle", 512, 512, ImPlotFlags_Equal);
  viewer->update_plot_line("basic/circle", "circle", xs_circle, ys_circle, ImPlotLineFlags_Loop);

  // ---- Plot types and flags ----
  // Line plots with flags (see ImPlotLineFlags in implot.h)
  viewer->setup_plot("styles/lines", 1024, 256);
  viewer->update_plot_line("styles/lines", "shaded", xs, ys_sin, ImPlotLineFlags_Shaded);
  viewer->update_plot_line("styles/lines", "segments", xs, ys_cos, ImPlotLineFlags_Segments);
  // Legend location and flags (see ImPlotLocation and ImPlotLegendFlags)
  viewer->setup_legend("styles/lines", ImPlotLocation_SouthWest, ImPlotLegendFlags_Horizontal);

  // Scatter and stairs plots
  viewer->setup_plot("styles/scatter_stairs", 1024, 256);
  viewer->update_plot_scatter("styles/scatter_stairs", "scatter", xs, ys_sin);
  viewer->update_plot_stairs("styles/scatter_stairs", "stairs_shaded", xs, ys_cos, ImPlotStairsFlags_Shaded);
  viewer->update_plot_stairs("styles/scatter_stairs", "stairs_prestep", xs, ys_sin, ImPlotStairsFlags_PreStep);

  // Data can also be given as Eigen vectors or arbitrary types with a function to extract values
  std::vector<Eigen::Vector2d> spiral;
  for (double t = 0.0; t < 6.0 * M_PI; t += 0.05) {
    spiral.emplace_back(t * std::cos(t), t * std::sin(t));
  }
  struct Sample {
    double stamp;
    double value;
  };
  std::vector<Sample> samples;
  for (double t = 0.0; t < 6.0 * M_PI; t += 0.5) {
    samples.push_back(Sample{t, 15.0 * std::sin(t)});
  }
  viewer->setup_plot("styles/custom_types", 512, 512, ImPlotFlags_Equal);
  viewer->update_plot_line("styles/custom_types", "Eigen::Vector2d", spiral);
  viewer->update_plot_scatter("styles/custom_types", "struct + lambda", samples, [](const Sample& s) { return Eigen::Vector2d(s.stamp, s.value); });

  // ---- Histograms ----
  std::mt19937 mt;
  std::normal_distribution<> ndist(0.0, 1.0);
  std::vector<double> gauss_x(5000);
  std::vector<double> gauss_y(5000);
  for (size_t i = 0; i < gauss_x.size(); i++) {
    gauss_x[i] = ndist(mt);
    gauss_y[i] = 0.5 * gauss_x[i] + 0.5 * ndist(mt);
  }

  // 1D histograms. The number of bins can be a positive number or an automatic binning method (ImPlotBin).
  viewer->setup_plot("histogram/1d", 1024, 256);
  viewer->update_plot_histogram("histogram/1d", "x (auto bins)", gauss_x);
  viewer->update_plot_histogram("histogram/1d", "y (20 bins)", gauss_y, 20);

  viewer->setup_plot("histogram/1d_density", 1024, 256);
  viewer->update_plot_histogram("histogram/1d_density", "density", gauss_x, 50, Eigen::Vector2d(-4.0, 4.0), ImPlotHistogramFlags_Density);
  viewer->update_plot_histogram("histogram/1d_density", "cumulative", gauss_x, 50, Eigen::Vector2d(-4.0, 4.0), ImPlotHistogramFlags_Density | ImPlotHistogramFlags_Cumulative);

  // 2D histogram
  viewer->setup_plot("histogram/2d", 512, 512);
  viewer->update_plot_histogram("histogram/2d", "xy", gauss_x, gauss_y, 50, 50);

  // ---- Realtime plots ----
  // AutoFit keeps the axes fitted to the data that are updated every frame
  viewer->setup_plot("realtime/signal", 1024, 200, 0, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
  viewer->setup_plot("realtime/noise", 1024, 200, 0, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
  // Axes with the same link ID are synchronized (try zooming the X axis after disabling the auto-fitting from the right click menu)
  viewer->link_plot_axis("realtime/signal", 0, ImAxis_X1);
  viewer->link_plot_axis("realtime/noise", 0, ImAxis_X1);

  // Display order of the plot groups (tabs)
  viewer->setup_plot_group_order("basic", 0);
  viewer->setup_plot_group_order("styles", 1);
  viewer->setup_plot_group_order("histogram", 2);
  viewer->setup_plot_group_order("realtime", 3);

  bool running = true;
  bool reset = false;
  viewer->register_ui_callback("plot_ui", [&] {
    ImGui::Begin("plot control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Checkbox("update realtime plots", &running);
    if (ImGui::Button("fit all plots")) {
      viewer->fit_all_plots();
    }
    ImGui::SameLine();
    if (ImGui::Button("reset realtime data")) {
      reset = true;
    }
    ImGui::End();
  });

  // Keep only the latest data
  const size_t max_num_data = 500;
  std::deque<double> stamps, signal, envelope, noise;

  double t = 0.0;
  while (viewer->spin_once()) {
    if (reset) {
      reset = false;
      stamps.clear();
      signal.clear();
      envelope.clear();
      noise.clear();
    }
    if (!running) {
      continue;
    }

    t += ImGui::GetIO().DeltaTime;
    stamps.push_back(t);
    signal.push_back(std::sin(t * 2.0) * std::exp(-0.2 * std::fmod(t, 10.0)));
    envelope.push_back(std::exp(-0.2 * std::fmod(t, 10.0)));
    noise.push_back(0.1 * ndist(mt));
    while (stamps.size() > max_num_data) {
      stamps.pop_front();
      signal.pop_front();
      envelope.pop_front();
      noise.pop_front();
    }

    // Plots with the same name and label are overwritten
    const std::vector<double> stamps_(stamps.begin(), stamps.end());
    viewer->update_plot_line("realtime/signal", "signal", stamps_, std::vector<double>(signal.begin(), signal.end()));
    viewer->update_plot_line("realtime/signal", "envelope", stamps_, std::vector<double>(envelope.begin(), envelope.end()));
    viewer->update_plot_scatter("realtime/noise", "noise", stamps_, std::vector<double>(noise.begin(), noise.end()));
  }

  return 0;
}
