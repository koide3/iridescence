#include <iostream>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

#include <portable-file-dialogs.h>

#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <guik/recent_files.hpp>

class KittiLoader {
public:
  KittiLoader(const std::string& dataset_path) : dataset_path(dataset_path) {
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

  size_t size() const {
    return num_frames;
  }

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

pcl::PointCloud<pcl::PointNormal>::Ptr preprocess(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_) {
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
  pcl::copyPointCloud(*cloud_, *cloud);

  // downsampling filter
  float downsample_resolution = 0.5f;
  pcl::ApproximateVoxelGrid<pcl::PointNormal> voxelgrid;
  voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
  voxelgrid.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>());
  voxelgrid.filter(*filtered);

  // normal estimation
  pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
  nest.setKSearch(15);
  nest.setInputCloud(filtered);
  nest.compute(*filtered);

  return filtered;
}

pcl::PointCloud<pcl::PointNormal>::Ptr fake_normal(pcl::PointCloud<pcl::PointNormal>::ConstPtr normals, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  pcl::search::KdTree<pcl::PointNormal> tree;
  tree.setInputCloud(normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  cloud_with_normals->resize(cloud->size());

#pragma omp parallel for
  for(int i = 0; i < cloud->size(); i++) {
    pcl::PointNormal pt;
    pt.getVector4fMap() = cloud->at(i).getVector4fMap();

    std::vector<int> k_indices(1);
    std::vector<float> sq_dists(1);
    tree.nearestKSearch(pt, 1, k_indices, sq_dists);

    pt.getNormalVector4fMap() = normals->at(k_indices[0]).getNormalVector4fMap();
    cloud_with_normals->at(i) = pt;
  }

  return cloud_with_normals;
}

int main(int argc, char** argv) {
  // "/your/kitti/path/sequences/00/velodyne"
  guik::RecentFiles recent_files("kitti_directory");
  std::string kitti_path = pfd::select_folder("Select KITTI directory that contains *.bin", recent_files.most_recent()).result();
  if(kitti_path.empty()) {
    return 0;
  }
  recent_files.push(kitti_path);

  KittiLoader kitti(kitti_path);

  // registration method
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
  icp.setMaxCorrespondenceDistance(2.0);

  // set initial frame as target
  auto target = preprocess(kitti.cloud(0));
  icp.setInputTarget(target);

  // viewer setting
  auto viewer = guik::LightViewer::instance();
  viewer->enable_normal_buffer();

  auto effect = std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size());
  viewer->set_screen_effect(effect);

  viewer->register_ui_callback("normal_texture", [&] { ImGui::Image((void*)effect->normal().id(), ImVec2(1920 / 4, 1080 / 4), ImVec2(0, 1), ImVec2(1, 0)); });

  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  for(int i = 1; i < kitti.size() && !viewer->closed(); i++) {
    effect->set_light(0, pose.translation() + Eigen::Vector3f::UnitZ(), Eigen::Vector4f::Ones(), Eigen::Vector2f(0.0f, 0.001f), 20.0f);
    if((i % 5) == 0) {
      effect->set_light(effect->num_lights(), pose.translation() + Eigen::Vector3f::UnitZ(), Eigen::Vector4f::Ones(), Eigen::Vector2f(0.0f, 0.01f), 20.0f);
    }

    // set the current frame as source
    auto source = kitti.cloud(i);
    auto downsampled = preprocess(source);
    auto source_with_normals = fake_normal(downsampled, source);
    icp.setInputSource(downsampled);

    pcl::PointCloud<pcl::PointNormal>::Ptr aligned(new pcl::PointCloud<pcl::PointNormal>);
    icp.align(*aligned);

    // accumulate pose
    pose = pose * icp.getFinalTransformation();
    icp.setInputTarget(icp.getInputSource());

    viewer->append_text((boost::format("%d : %.3f s : %d pts   fitness_score %.3f") % i % ImGui::GetTime() % downsampled->size() % icp.getFitnessScore()).str());

    auto cloud_buffer = glk::create_point_cloud_buffer(*source_with_normals);
    viewer->update_drawable("current_frame", cloud_buffer, guik::FlatColor(1.0f, 0.5f, 0.0f, 1.0f, pose).add("point_scale", 3.0f));
    viewer->update_drawable("frame_" + std::to_string(i), cloud_buffer, guik::Rainbow(pose));
    viewer->update_drawable("coord_" + std::to_string(i), glk::Primitives::primitive_ptr(glk::Primitives::COORDINATE_SYSTEM), guik::VertexColor(pose));
    viewer->lookat(pose.translation());

    if(!viewer->spin_once()) {
      break;
    }
  }

  return 0;
}