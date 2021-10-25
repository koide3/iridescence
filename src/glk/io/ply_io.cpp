#include <glk/io/ply_io.hpp>

#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>

#include <glk/mesh_utils.hpp>
#include <glk/console_colors.hpp>

namespace glk {

using namespace glk::console;

std::shared_ptr<PLYData> load_ply_ascii(const std::string& filename) {
  std::ifstream ifs(filename);
  if(!ifs) {
    std::cerr << bold_red << "error: failed to open " << filename << reset << std::endl;
    return nullptr;
  }

  int num_vertices = 0;
  int num_faces = 0;
  std::vector<std::string> properties;
  while(!ifs.eof()) {
    std::string line;
    std::getline(ifs, line);

    if(line.empty()) {
      continue;
    }

    std::stringstream sst(line);
    std::string token;

    if(line.find("element vertex") != std::string::npos) {
      sst >> token >> token >> num_vertices;
    }
    if(line.find("element face") != std::string::npos) {
      sst >> token >> token >> num_faces;
    }

    if(line.find("property float32") != std::string::npos) {
      std::string property;
      sst >> token >> token >> property;
      properties.push_back(property);
    }

    if(line.find("end") != std::string::npos) {
      break;
    }
  }

  std::shared_ptr<PLYData> ply(new PLYData);

  ply->vertices.resize(num_vertices);
  for(int i = 0; i < num_vertices; i++) {
    std::string line;
    std::getline(ifs, line);

    std::stringstream sst(line);
    sst >> ply->vertices[i][0] >> ply->vertices[i][1] >> ply->vertices[i][2];
  }

  ply->indices.resize(num_faces * 3);
  for(int i = 0; i < num_faces; i++) {
    std::string line;
    std::getline(ifs, line);

    int faces = 0;
    std::stringstream sst(line);
    sst >> faces >> ply->indices[i * 3 + 2] >> ply->indices[i * 3 + 1] >> ply->indices[i * 3];

    if(faces != 3) {
      std::cerr << bold_red << "error : only faces with three vertices are supported!!" << reset << std::endl;
    }
  }

  NormalEstimater nest(ply->vertices, ply->indices);
  ply->normals = nest.normals;

  return ply;
}

bool write_ply_header(std::ofstream& ofs, const PLYData& ply, const std::string& type = "ascii") {
  ofs << "ply" << std::endl;
  ofs << "format " << type << " 1.0" << std::endl;
  ofs << "element vertex " << ply.vertices.size() << std::endl;
  ofs << "property float32 x" << std::endl;
  ofs << "property float32 y" << std::endl;
  ofs << "property float32 z" << std::endl;

  if(ply.colors.size()) {
    ofs << "property float32 red" << std::endl;
    ofs << "property float32 green" << std::endl;
    ofs << "property float32 blue" << std::endl;
    ofs << "property float32 alpha" << std::endl;
  }

  if(ply.intensities.size()) {
    ofs << "property float32 intensity" << std::endl;
  }

  if(ply.indices.size()) {
    ofs << "element face " << ply.indices.size() / 3 << std::endl;
    ofs << "property list int32 int32 vertex_indices" << std::endl;
  }
  ofs << "end_header" << std::endl;

  return true;
}

bool save_ply_ascii(const std::string& filename, const PLYData& ply) {
  std::ofstream ofs(filename);
  if(!ofs) {
    std::cerr << bold_red << "error: failed to open " << filename << std::endl;
    return false;
  }

  write_ply_header(ofs, ply);

  for(int i = 0; i < ply.vertices.size(); i++) {
    ofs << ply.vertices[i][0] << " " << ply.vertices[i][1] << " " << ply.vertices[i][2];

    if(ply.colors.size()) {
      ofs << " " << ply.colors[i][0] << " " << ply.colors[i][1] << " " << ply.colors[i][2] << " " << ply.colors[i][3];
    }

    if(ply.intensities.size()) {
      ofs << " " << ply.intensities[i];
    }

    ofs << std::endl;
  }

  for(int i = 0; i < ply.indices.size() / 3; i++) {
    ofs << 3 << " " << ply.indices[i * 3] << " " << ply.indices[i * 3 + 1] << " " << ply.indices[i * 3 + 2] << std::endl;
  }

  return true;
}

struct PLYFace {
  int32_t num_indices;
  int32_t indices[3];
};

bool save_ply_binary(const std::string& filename, const PLYData& ply) {
  std::ofstream ofs(filename, std::ios::binary);
  if(!ofs) {
    std::cerr << bold_red << "error: failed to open " << filename << std::endl;
    return false;
  }

  write_ply_header(ofs, ply, "binary_little_endian");

  int num_fields = 3;
  if(ply.colors.size()) {
    num_fields += 4;
  }
  if(ply.intensities.size()) {
    num_fields += 1;
  }

  Eigen::Matrix<float, -1, -1, Eigen::RowMajor> data(ply.vertices.size(), num_fields);
  for(int i = 0; i < ply.vertices.size(); i++) {
    int begin = 0;
    for(int j = 0; j < 3; j++) {
      data(i, begin + j) = ply.vertices[i][j];
    }
    begin += 3;

    if(ply.colors.size()) {
      for(int j = 0; j < 4; j++) {
        data(i, begin + j) = ply.colors[i][j];
      }
      begin += 4;
    }

    if(ply.intensities.size()) {
      data(i, begin) = ply.intensities[i];
      begin += 1;
    }
  }

  ofs.write(reinterpret_cast<const char*>(data.data()), sizeof(float) * num_fields * ply.vertices.size());

  std::vector<PLYFace> faces(ply.indices.size() / 3);
  for(int i = 0; i < ply.indices.size() / 3; i++) {
    faces[i].num_indices = 3;
    for(int j = 0; j < 3; j++) {
      faces[i].indices[j] = ply.indices[i * 3 + j];
    }
  }

  ofs.write(reinterpret_cast<const char*>(faces.data()), sizeof(PLYFace) * faces.size());

  return true;
}

template<typename T, int D>
bool save_ply_binary(const std::string& filename, const Eigen::Matrix<T, D, 1>* points, int num_points) {
  PLYData ply;
  ply.vertices.resize(num_points);
  for(int i = 0; i < num_points; i++) {
    ply.vertices[i] = points[i].template head<3>().template cast<float>();
  }

  return save_ply_binary(filename, ply);
}

template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<float, 3, 1>* points, int num_points);
template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<float, 4, 1>* points, int num_points);
template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<double, 3, 1>* points, int num_points);
template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<double, 4, 1>* points, int num_points);

}  // namespace glk
