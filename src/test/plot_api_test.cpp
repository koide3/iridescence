#include <numeric>
#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer.hpp>

void plot_api_test() {
  std::vector<double> xs;
  for (double x = 0.0; x <= 2.0 * M_PI; x += 0.01) {
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

  auto viewer = guik::viewer();

  viewer->update_plot_line("plots", "sin", ys);
  viewer->update_plot_line("plots", "sin", xs, ys);
  viewer->update_plot_line("plots", "sin2", xys);
  viewer->update_plot_line("plots", "sin2", xys3);
  viewer->update_plot_line("plots2", "sin3", indices, [&](int i) { return xys[i].y(); });
  viewer->update_plot_line("plots", "sin4", indices, [&](int i) { return xys[i]; });

  viewer->update_plot_line("plots", "sinf", ysf);
  viewer->update_plot_line("plots", "sinf", xsf, ysf);
  viewer->update_plot_line("plots", "sin2f", xysf);
}

void plot_api_test_async() {
  std::vector<double> xs;
  for (double x = 0.0; x <= 2.0 * M_PI; x += 0.01) {
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

  auto viewer = guik::async_viewer();

  viewer->update_plot_line("plots", "sin", ys);
  viewer->update_plot_line("plots", "sin", xs, ys);
  viewer->update_plot_line("plots", "sin2", xys);
  viewer->update_plot_line("plots", "sin2", xys3);
  viewer->update_plot_line("plots2", "sin3", indices, [&](int i) { return xys[i].y(); });
  viewer->update_plot_line("plots", "sin4", indices, [&](int i) { return xys[i]; });

  viewer->update_plot_line("plots", "sinf", ysf);
  viewer->update_plot_line("plots", "sinf", xsf, ysf);
  viewer->update_plot_line("plots", "sin2f", xysf);
}

int main(int argc, char** argv) {
  return 0;
}