#include <glk/io/ply_io.hpp>

#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>

#include <glk/mesh_utils.hpp>
#include <glk/console_colors.hpp>

namespace glk {

using namespace glk::console;

PLYMetaData::PropertyType parse_property_type(const std::string& type) {
  if(type == "char" || type == "int8") {
    return PLYMetaData::PropertyType::CHAR;
  } else if(type == "uchar" || type == "uint8") {
    return PLYMetaData::PropertyType::UCHAR;
  } else if(type == "short" || type == "int16") {
    return PLYMetaData::PropertyType::SHORT;
  } else if(type == "ushort" || type == "uint16") {
    return PLYMetaData::PropertyType::USHORT;
  } else if(type == "int" || type == "int32") {
    return PLYMetaData::PropertyType::INT;
  } else if(type == "uint" || type == "uint32") {
    return PLYMetaData::PropertyType::UINT;
  } else if(type == "float" || type == "float32") {
    return PLYMetaData::PropertyType::FLOAT;
  } else if(type == "double" || type == "float64") {
    return PLYMetaData::PropertyType::DOUBLE;
  } else {
    std::cerr << console::bold_red << "error: unknown ply property type " << type << console::reset << std::endl;
  }
}

int property_bytes(PLYMetaData::PropertyType prop) {
  switch(prop) {
    case PLYMetaData::PropertyType::CHAR:
    case PLYMetaData::PropertyType::UCHAR:
      return 1;
    case PLYMetaData::PropertyType::SHORT:
    case PLYMetaData::PropertyType::USHORT:
      return 2;
    case PLYMetaData::PropertyType::INT:
    case PLYMetaData::PropertyType::UINT:
    case PLYMetaData::PropertyType::FLOAT:
      return 4;
    case PLYMetaData::PropertyType::DOUBLE:
      return 8;
  }
}

template<typename T>
T ply_data_cast(const char* data, PLYMetaData::PropertyType type, double int_scale = 1.0) {
  switch(type) {
    case PLYMetaData::PropertyType::CHAR:
      return static_cast<T>((*reinterpret_cast<const int8_t*>(data)) * int_scale);
    case PLYMetaData::PropertyType::UCHAR:
      return static_cast<T>((*reinterpret_cast<const uint8_t*>(data)) * int_scale);
    case PLYMetaData::PropertyType::SHORT:
      return static_cast<T>((*reinterpret_cast<const int16_t*>(data)) * int_scale);
    case PLYMetaData::PropertyType::USHORT:
      return static_cast<T>((*reinterpret_cast<const uint16_t*>(data)) * int_scale);
    case PLYMetaData::PropertyType::INT:
      return static_cast<T>((*reinterpret_cast<const int32_t*>(data)) * int_scale);
    case PLYMetaData::PropertyType::UINT:
      return static_cast<T>((*reinterpret_cast<const uint32_t*>(data)) * int_scale);
    case PLYMetaData::PropertyType::FLOAT:
      return static_cast<T>(*reinterpret_cast<const float*>(data));
    case PLYMetaData::PropertyType::DOUBLE:
      return static_cast<T>(*reinterpret_cast<const double*>(data));
  }
}

template<typename OutputIterator, typename Func>
void transform(const char* vertex_buffer, int vertex_step, int num_vertices, int property_offset, OutputIterator output_itr, Func func) {
  for(int i = 0; i < num_vertices; i++) {
    const char* ptr = vertex_buffer + vertex_step * i + property_offset;
    func(ptr, *output_itr);
    output_itr++;
  }
}

std::shared_ptr<PLYData> load_ply_body_binary(std::ifstream& ifs, const PLYMetaData& meta_data) {
  if(meta_data.format.find("big_endian") != std::string::npos) {
    std::cerr << console::bold_red << "error: big endian is not supported!!" << console::reset << std::endl;
    return nullptr;
  }

  std::vector<int> property_offsets = {0};
  for(const auto& prop : meta_data.vertex_properties) {
    property_offsets.push_back(property_offsets.back() + property_bytes(prop.second));
  }

  const int vertex_step = property_offsets.back();
  std::vector<char> vertex_buffer(vertex_step * meta_data.num_vertices);
  ifs.read(vertex_buffer.data(), vertex_buffer.size());

  std::shared_ptr<PLYData> ply(new PLYData);
  for(int i = 0; i < meta_data.vertex_properties.size(); i++) {
    const auto& prop_name = meta_data.vertex_properties[i].first;
    const auto prop_type = meta_data.vertex_properties[i].second;
    const int prop_offset = property_offsets[i];

    // xyz
    if(prop_name == "x") {
      ply->vertices.resize(meta_data.num_vertices);
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->vertices.begin(),
                [=](const char* data, Eigen::Vector3f& pt) { pt.x() = ply_data_cast<float>(data, prop_type); });
    } else if(prop_name == "y") {
      ply->vertices.resize(meta_data.num_vertices);
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->vertices.begin(),
                [=](const char* data, Eigen::Vector3f& pt) { pt.y() = ply_data_cast<float>(data, prop_type); });
    } else if(prop_name == "z") {
      ply->vertices.resize(meta_data.num_vertices);
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->vertices.begin(),
                [=](const char* data, Eigen::Vector3f& pt) { pt.z() = ply_data_cast<float>(data, prop_type); });
    }
    // normals
    else if(prop_name == "nx") {
      ply->normals.resize(meta_data.num_vertices);
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->normals.begin(),
                [=](const char* data, Eigen::Vector3f& pt) { pt.x() = ply_data_cast<float>(data, prop_type); });
    } else if(prop_name == "ny") {
      ply->normals.resize(meta_data.num_vertices);
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->normals.begin(),
                [=](const char* data, Eigen::Vector3f& pt) { pt.y() = ply_data_cast<float>(data, prop_type); });
    } else if(prop_name == "nz") {
      ply->normals.resize(meta_data.num_vertices);
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->normals.begin(),
                [=](const char* data, Eigen::Vector3f& pt) { pt.z() = ply_data_cast<float>(data, prop_type); });
    }
    // color
    else if(prop_name == "r" || prop_name == "red") {
      ply->colors.resize(meta_data.num_vertices, Eigen::Vector4f::Zero());
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->colors.begin(),
                [=](const char* data, Eigen::Vector4f& pt) { pt[0] = ply_data_cast<float>(data, prop_type, 1.0/255.0); });
    } else if(prop_name == "g" || prop_name == "green") {
      ply->colors.resize(meta_data.num_vertices, Eigen::Vector4f::Zero());
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->colors.begin(),
                [=](const char* data, Eigen::Vector4f& pt) { pt[1] = ply_data_cast<float>(data, prop_type, 1.0 / 255.0); });
    } else if(prop_name == "b" || prop_name == "blue") {
      ply->colors.resize(meta_data.num_vertices, Eigen::Vector4f::Zero());
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->colors.begin(),
                [=](const char* data, Eigen::Vector4f& pt) { pt[2] = ply_data_cast<float>(data, prop_type, 1.0 / 255.0); });
    } else if(prop_name == "a" || prop_name == "alpha") {
      ply->colors.resize(meta_data.num_vertices, Eigen::Vector4f::Zero());
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->colors.begin(),
                [=](const char* data, Eigen::Vector4f& pt) { pt[3] = ply_data_cast<float>(data, prop_type, 1.0 / 255.0); });
    }
    // intensity
    else if(prop_name == "intensity" || prop_name == "scalar_Intensity" || prop_name == "scalar_intensity") {
      ply->intensities.resize(meta_data.num_vertices);
      transform(vertex_buffer.data(), vertex_step, meta_data.num_vertices, prop_offset, ply->intensities.begin(),
                [=](const char* data, float& pt) { pt = ply_data_cast<float>(data, prop_type); });
    }
  }

  if(meta_data.num_faces && meta_data.face_properties.empty()) {
    std::cerr << console::yellow << "warning: face properties dont exist!!" << console::reset << std::endl;
  }

  if(meta_data.face_properties.size() == 2) {
    const int face_size = property_bytes(meta_data.face_properties[0]) + property_bytes(meta_data.face_properties[1]) * 3;
  }

  return ply;
}

std::shared_ptr<PLYData> load_ply_body_ascii(std::ifstream& ifs, const PLYMetaData& meta_data) {}

std::shared_ptr<PLYData> load_ply(const std::string& filename) {
  std::ifstream ifs(filename, std::ios::binary);
  if(!ifs) {
    std::cerr << bold_red << "error: failed to open " << filename << reset << std::endl;
    return nullptr;
  }

  PLYMetaData meta_data;

  while(!ifs.eof()) {
    std::string line;
    std::getline(ifs, line);

    if(line.empty()) {
      continue;
    }

    std::stringstream sst(line);
    std::string token;

    if(line.find("format") != std::string::npos) {
      sst >> token >> meta_data.format;
    }

    if(line.find("element vertex") != std::string::npos) {
      sst >> token >> token >> meta_data.num_vertices;
    }
    if(line.find("element face") != std::string::npos) {
      sst >> token >> token >> meta_data.num_faces;
    }

    if(line.find("property") != std::string::npos) {
      std::string property;
      std::string type;
      sst >> token >> type;

      if(type != "list") {
        sst >> property;
        PLYMetaData::PropertyType prop_type = parse_property_type(type);
        meta_data.vertex_properties.push_back(std::make_pair(property, prop_type));
      } else {
        std::string num_type;
        std::string index_type;
        sst >> num_type >> index_type;

        std::cout << num_type << " " << index_type << std::endl;

      }
    }

    if(line.find("end_header") != std::string::npos) {
      break;
    }
  }

  if(meta_data.format.find("ascii") != std::string::npos) {
    return load_ply_body_ascii(ifs, meta_data);
  }

  if(meta_data.format.find("binary") != std::string::npos) {
    return load_ply_body_binary(ifs, meta_data);
  }

  std::cerr << console::bold_red << "error: unknown ply format " << meta_data.format << console::reset << std::endl;
  return nullptr;
}

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
