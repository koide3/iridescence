#include <fstream>
#include <iostream>
#include <boost/format.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <glk/effects/screen_space_splatting.hpp>

#include <guik/model_control.hpp>
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

  std::shared_ptr<glk::ScreenSpaceLighting> effect(new glk::ScreenSpaceLighting(Eigen::Vector2i(2560, 1440), true));

  viewer->set_screen_effect(effect);
  viewer->shader_setting().add("point_size", 0.0f);
  viewer->shader_setting().add("point_size_offset", 1.0f);

  Eigen::Matrix4f init_light_matrix = Eigen::Matrix4f::Identity();
  init_light_matrix.block<3, 1>(0, 3) = Eigen::Vector3f(10.0f, 10.0f, 5.0f);

  guik::ModelControl model_control("light_control", init_light_matrix);

  float roughness = 0.1f;
  effect->set_roughness(roughness);

  viewer->register_ui_callback("light_pos", [&] {
    model_control.draw_gizmo_ui();
    model_control.draw_gizmo(0, 0, viewer->canvas_size()[0], viewer->canvas_size()[1], viewer->view_matrix(), viewer->projection_matrix());

    effect->set_light(0, model_control.model_matrix().block<3, 1>(0, 3), Eigen::Vector4f::Ones() * 2.0f);

    if(ImGui::DragFloat("roughness", &roughness, 0.01f, 0.01f, 2.0f)) {
      effect->set_roughness(roughness);
    }
  });

  // viewer->set_screen_effect(std::shared_ptr<glk::ScreenSpaceLighting>(new glk::ScreenSpaceLighting(Eigen::Vector2i(2560, 1440))));

  load("/home/koide/dump");

  viewer->spin();
  return 0;
}