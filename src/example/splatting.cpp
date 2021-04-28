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

  float roughness = 0.1f;
  effect->set_roughness(roughness);

  viewer->register_ui_callback("roughness", [&] {
    if(ImGui::DragFloat("roughness", &roughness, 0.001f, 0.001f, 2.0f)) {
      effect->set_roughness(roughness);
    }
  });

  // viewer->set_screen_effect(std::shared_ptr<glk::ScreenSpaceLighting>(new glk::ScreenSpaceLighting(Eigen::Vector2i(2560, 1440))));

  load("/home/koide/dump");

  const int num_lights = 8;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> light_colors(num_lights);
  for(int i = 0; i < num_lights; i++) {
    light_colors[i] = glk::colormap_categoricalf(glk::COLORMAP::TURBO, i, num_lights);
  }

  double t = 0.0;
  while(viewer->spin_once()) {
    t += ImGui::GetIO().DeltaTime;
    for(int i = 0; i < num_lights; i++) {
      double theta = t + i * 2.0 * M_PI / num_lights;
      Eigen::Vector3f light_pos(15.0 * std::cos(theta), 15.0 * std::sin(theta), 0.0f);
      light_pos += Eigen::Vector3f(20.0f, 22.0f, 3.0f);
      Eigen::Vector2f attenuation(0.0f, 0.001f);
      float max_distance = 200.0f;
      effect->set_light(i, light_pos, light_colors[i], attenuation, max_distance);

      Eigen::Affine3f model_matrix = Eigen::Translation3f(light_pos) * Eigen::UniformScaling<float>(0.25f) * Eigen::Isometry3f::Identity();
      viewer->update_drawable("light_" + std::to_string(i), glk::Primitives::sphere(),
                              guik::FlatColor(light_colors[i], model_matrix).make_transparent());
    }
  }
  return 0;
}