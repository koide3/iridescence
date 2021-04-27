#include <fstream>
#include <iostream>
#include <boost/format.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <glk/effects/screen_space_splatting.hpp>
#include <guik/viewer/light_viewer.hpp>

void load(const std::string& dump_path) {
  std::ifstream ifs(dump_path + "/graph.txt");
  if(!ifs) {
    std::cerr << "failed to open " << dump_path + "/graph.txt" << std::endl;
    abort();
  }

  std::string token;
  int num_frames, num_all_frames;
  ifs >> token >> num_frames >> token >> num_all_frames;
  std::cout << "num_frames:" << num_frames << std::endl;

  for(int i = 0; i < num_frames; i++) {
    Eigen::Isometry3f submap_pose = Eigen::Isometry3f::Identity();
    ifs >> token;

    Eigen::Vector3f trans;
    Eigen::Quaternionf quat;

    ifs >> trans.x() >> trans.y() >> trans.z() >> quat.x() >> quat.y() >> quat.z() >> quat.w();
    submap_pose.translation() = trans;
    submap_pose.linear() = quat.toRotationMatrix();

    std::ifstream info_ifs((boost::format("%s/%06d/frame_info.txt") % dump_path % i).str());
    std::ifstream points_ifs((boost::format("%s/%06d/frame_points.bin") % dump_path % i).str(), std::ios::binary);
    if(!info_ifs || !points_ifs) {
      std::cerr << "failed to open points file" << std::endl;
      abort();
    }

    int num_points;
    info_ifs >> token >> token >> token >> num_points;

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points(num_points);
    points_ifs.read(reinterpret_cast<char*>(points.data()), sizeof(Eigen::Vector3f) * num_points);

    auto viewer = guik::LightViewer::instance();
    viewer->update_drawable("frame_" + std::to_string(i), std::make_shared<glk::PointCloudBuffer>(points), guik::Rainbow(submap_pose));
  }
}

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(2560, 1440));
  viewer->show_info_window();
  viewer->set_draw_xy_grid(false);
  viewer->set_screen_effect(std::shared_ptr<glk::ScreenSpaceSplatting>(new glk::ScreenSpaceSplatting()));

  load("/home/koide/dump");

  viewer->spin();
  return 0;
}