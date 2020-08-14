#include <iostream>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

class KittiLoader {
public:
  KittiLoader(const std::string& dataset_path)
  : dataset_path(dataset_path)
  {
    for(num_frames = 0;; num_frames++) {
      std::string filename = (boost::format("%s/%06d.bin") % dataset_path % num_frames).str();
      if(!boost::filesystem::exists(filename)) {
        break;
      }
    }

    if(num_frames == 0) {
      std::cerr << "error: no files in " << dataset_path << std::endl;
    }
  }
  ~KittiLoader() {}

  size_t size() const { return num_frames; }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud(size_t i) const {
    std::string filename = (boost::format("%s/%06d.bin") % dataset_path % i).str();
    FILE* file = fopen(filename.c_str(), "rb");
    if(!file) {
      std::cerr << "error: failed to load " << filename << std::endl;
      return nullptr;
    }

    std::vector<float> buffer(1000000);
    size_t num_points = fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(num_points);

    for(int i = 0; i < num_points; i++) {
      auto& pt = cloud->at(i);
      pt.x = buffer[i * 4];
      pt.y = buffer[i * 4 + 1];
      pt.z = buffer[i * 4 + 2];
      // pt.intensity = buffer[i * 4 + 3];
    }

    return cloud;
  }

private:
  int num_frames;
  std::string dataset_path;
};


int main(int argc, char** argv) {
  if(argc < 2) {
    std::cout << "usage: light_viewer_kitti /your/kitti/path/sequences/00/velodyne" << std::endl;
    return 0;
  }

  KittiLoader kitti(argv[1]);

  // downsampling filter
  float downsample_resolution = 1.0f;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

  // registration method
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setMaxCorrespondenceDistance(1.0);

  // set initial frame as target
  voxelgrid.setInputCloud(kitti.cloud(0));
  auto target = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxelgrid.filter(*target);
  gicp.setInputTarget(target);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  // viewer setting
  auto viewer = guik::LightViewer::instance();

  bool step_execution = true;
  viewer->register_ui_callback("registration config", [&]() {
    if(ImGui::Button("clear map")) {
      viewer->clear_drawables([](const std::string& name){ return name.find("frame_") != std::string::npos; });
    }

    ImGui::SameLine();
    if(ImGui::Button("clear all")) {
      viewer->clear_drawables();
    }

    if(ImGui::DragFloat("downsample resolution", &downsample_resolution, 0.05f, 0.1f, 2.0f)) {
      voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    }
  });

  for(int i=1; i<kitti.size() && !viewer->closed(); i++) {
    // set the current frame as source
    voxelgrid.setInputCloud(kitti.cloud(i));
    auto source = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxelgrid.filter(*source);
    gicp.setInputSource(source);

    auto aligned = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    gicp.align(*aligned);

    // accumulate pose
    pose = pose * gicp.getFinalTransformation().cast<double>();
    gicp.setInputTarget(gicp.getInputSource());

    viewer->append_text((boost::format("%d : %.3f s : %d pts   fitness_score %.3f") % i % ImGui::GetTime() % source->size() % gicp.getFitnessScore()).str());

    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(kitti.cloud(i));
    viewer->update_drawable("current_frame", cloud_buffer, guik::FlatColor(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f), pose.cast<float>().matrix()).add("point_scale", 10.0f));
    viewer->update_drawable("frame_" + std::to_string(i), cloud_buffer, guik::Rainbow(pose.cast<float>().matrix()));
    viewer->update_drawable("coord_" + std::to_string(i), glk::Primitives::primitive_ptr(glk::Primitives::COORDINATE_SYSTEM), guik::VertexColor(pose.cast<float>().cast<float>().matrix()));
    viewer->lookat(pose.translation().cast<float>());

    // step execution
    bool step_clicked = false;
    viewer->register_ui_callback("step execution config", [&]() {
      ImGui::Checkbox("##step", &step_execution);

      ImGui::SameLine();
      if(ImGui::Button("step execution")) {
        step_clicked = true;
      }
    });

    while(viewer->spin_once() && step_execution && !step_clicked) {}
  }

  return 0;
}