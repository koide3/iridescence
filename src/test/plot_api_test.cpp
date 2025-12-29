#include <numeric>
#include <gtest/gtest.h>
#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer.hpp>

void plot_api_test() {
  std::vector<double> xs;
  for (double x = 0.0; x <= 2.0 * M_PI; x += 0.1) {
    xs.emplace_back(x);
  }

  std::vector<double> ys(xs.size());
  std::transform(xs.begin(), xs.end(), ys.begin(), [](double x) { return std::sin(x); });

  std::vector<Eigen::Vector2d> xys(xs.size());
  std::vector<Eigen::Vector3d> xys3(xs.size());
  for (int i = 0; i < xs.size(); i++) {
    xys[i] = Eigen::Vector2d(xs[i], ys[i]);
    xys3[i] = Eigen::Vector3d(xs[i], ys[i], 0.0);
  }

  std::vector<float> xsf(xs.begin(), xs.end());
  std::copy(xs.begin(), xs.end(), xsf.begin());

  std::vector<float> ysf(ys.begin(), ys.end());
  std::copy(ys.begin(), ys.end(), ysf.begin());

  std::vector<Eigen::Vector2f> xysf(xys.size());
  std::transform(xys.begin(), xys.end(), xysf.begin(), [](const Eigen::Vector2d& xy) { return xy.cast<float>(); });

  std::vector<int> indices(xs.size());
  std::iota(indices.begin(), indices.end(), 0);

  auto viewer = guik::viewer({-1, -1}, false);

  viewer->update_plot_line("plots", "line2", xs, ys);
  viewer->update_plot_line("plots", "line3", xys);
  viewer->update_plot_line("plots", "line4", xys3);
  viewer->update_plot_line("plots", "line6", indices, [&](int i) { return xys[i]; });

  viewer->update_plot_line("plots2", "line1", ys);
  viewer->update_plot_line("plots2", "line5", indices, [&](int i) { return xys[i].y(); });

  viewer->update_plot_scatter("plots", "scatter2", xs, ys);
  viewer->update_plot_scatter("plots", "scatter3", xys);
  viewer->update_plot_scatter("plots", "scatter4", xys3);
  viewer->update_plot_scatter("plots", "scatter6", indices, [&](int i) { return xys[i]; });

  viewer->update_plot_scatter("plots2", "scatter1", ys);
  viewer->update_plot_scatter("plots2", "scatter5", indices, [&](int i) { return xys[i].y(); });

  viewer->update_plot_line("plotsf", "line2f", xsf, ysf);
  viewer->update_plot_line("plotsf", "line3f", xysf);
  viewer->update_plot_scatter("plotsf", "scatter2f", xsf, ysf);
  viewer->update_plot_scatter("plotsf", "scatter3f", xysf);

  viewer->update_plot_line("plots2f", "line1f", ysf);
  viewer->update_plot_scatter("plots2f", "scatter1f", ysf);

  auto now = std::chrono::high_resolution_clock::now();
  while (std::chrono::high_resolution_clock::now() - now < std::chrono::milliseconds(100)) {
    viewer->spin_once();
  }
}

TEST(PlotTest, APITest) {
  plot_api_test();
}