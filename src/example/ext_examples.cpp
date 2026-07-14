#include <chrono>
#include <fstream>
#include <filesystem>
#include <glk/path.hpp>
#include <glk/voxelmap.hpp>
#include <glk/profiler.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  const std::string path = "/home/koide/datasets/occupancy_grid_map";
  std::vector<std::string> filenames;
  for (const auto path : std::filesystem::directory_iterator(path)) {
    if (path.path().extension() == ".bin") {
      filenames.push_back(path.path());
    }
  }

  std::sort(filenames.begin(), filenames.end());

  std::vector<std::vector<Eigen::Vector3i>> all_cells;
  all_cells.reserve(filenames.size());

  for (const auto& filename : filenames) {
    std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
    if (!ifs) {
      std::cerr << "Failed to open file: " << filename << std::endl;
      continue;
    }

    const size_t num_cells = ifs.tellg() / sizeof(Eigen::Vector3i);
    ifs.seekg(0, std::ios::beg);

    std::vector<Eigen::Vector3i> cells(num_cells);
    ifs.read(reinterpret_cast<char*>(cells.data()), num_cells * sizeof(Eigen::Vector3i));
    all_cells.emplace_back(std::move(cells));

    std::cout << "loaded " << num_cells << " cells from " << filename << std::endl;
  }

  auto viewer = guik::viewer();
  viewer->disable_vsync();

  const auto t1 = std::chrono::high_resolution_clock::now();
  for (const auto& cells : all_cells) {
    glk::GLRealProfiler prof("voxelmap");
    prof.add("create");
    glk::MeshRenderingOptions options;
    options.set_edge_color(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));

    auto voxelmap = std::make_shared<glk::VoxelMap2>(cells, 0.5, options);

    prof.add("update");
    viewer->update_drawable("voxelmap", voxelmap, guik::Rainbow());

    prof.add("render1");
    viewer->spin_once();
    prof.add("render2");
    viewer->spin_once();
    prof.add("render3");
    viewer->spin_once();

    // break;
  }
  const auto t2 = std::chrono::high_resolution_clock::now();
  const double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.0;
  std::cerr << "elapsed time: " << elapsed_time * 1000.0 << " [msec]   average: " << (elapsed_time / all_cells.size()) * 1000.0 << " [msec/frame]" << std::endl;

  // while (viewer->ok()) {
  //   glk::GLRealProfiler prof("render");
  //   viewer->spin_once();
  //   prof.add("done");
  // }

  viewer->spin();

  return 0;
}