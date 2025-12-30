#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <portable-file-dialogs.h>

#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <glk/primitives/primitives.hpp>

#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  std::vector<std::string> filenames;

  viewer->register_ui_callback("cloud_loader", [&]() {
    if (ImGui::Button("load")) {
      std::vector<std::string> results = pfd::open_file("choose PCD file").result();
      if (!results.empty()) {
        filenames.push_back(results[0]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(results[0], *cloud);

        auto cloud_buffer = glk::create_point_cloud_buffer(*cloud);
        viewer->update_drawable(results[0], cloud_buffer, guik::FlatColor(Eigen::Vector4f::Random()));

        // *** example usage ***
        // construct PointCloudBuffer from raw float pointer
        // auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(cloud->front().data, sizeof(pcl::PointXYZ), cloud->size());

        // use rainbow color (height encoding)
        // viewer->update_drawable(results[0], cloud_buffer, guik::Rainbow());

        // with some transformation and bigger points
        // viewer->update_drawable(results[0], cloud_buffer, guik::FlatColor(Eigen::Vector4f::Random(), Eigen::Translation3f(1.0f, 1.0f, 1.0f)).add("point_scale", 5.0f));

        // with point color
        // std::vector<Eigen::Vector4f> colors(cloud->size());
        // for(int i = 0; i < cloud->size(); i++) {
        //   colors[i] = cloud->at(i).getVector4fMap() * 0.1f;
        // }
        // cloud_buffer->add_color(colors[0].data(), sizeof(Eigen::Vector4f), colors.size());
        // viewer->update_drawable(results[0], cloud_buffer, guik::VertexColor());

        // another way to create colored points with pcl::PointXYZRGBA
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // colored = ...
        // auto cloud_buffer = glk::create_colored_point_cloud_buffer(*colored);
        // viewer->update_drawable(results[0], cloud_buffer, guik::VertexColor());
      }
    }

    for (int i = 0; i < filenames.size(); i++) {
      std::string button_name = "remove##" + std::to_string(i);
      std::string filename = filenames[i];
      if (ImGui::Button(button_name.c_str())) {
        viewer->remove_drawable(filenames[i]);
        filenames.erase(filenames.begin() + i);
      }

      ImGui::SameLine();
      ImGui::Text("%s", filename.c_str());
    }
  });

  viewer->spin();

  return 0;
}