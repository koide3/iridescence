#include <glk/io/pcd_io.hpp>

#include <fstream>
#include <iostream>
#include <boost/algorithm/string.hpp>

namespace glk {

struct PCDMetaData {
  PCDMetaData() {
    version = 0.0;
    width = 0;
    height = 0;
    points = 0;
    ascii = false;
  }

  bool read(std::istream& ist) {
    std::string line;
    while (!ist.eof()) {
      std::getline(ist, line);
      if (!line.empty() && line[0] == '#') {
        continue;
      }

      std::stringstream sst(line);
      std::string token;
      sst >> token;

      if (token == "VERSION") {
        sst >> version;
      } else if (token == "FIELDS") {
        std::deque<std::string> names;
        boost::split(names, line, boost::is_any_of(" "));
        names.pop_front();

        if (fields.size() != names.size()) {
          fields.resize(names.size());
        }

        for (int i = 0; i < names.size(); i++) {
          std::get<0>(fields[i]) = names[i];
        }
      } else if (token == "SIZE") {
        std::deque<std::string> sizes;
        boost::split(sizes, line, boost::is_any_of(" "));
        sizes.pop_front();

        if (fields.size() != sizes.size()) {
          fields.resize(sizes.size());
        }

        for (int i = 0; i < sizes.size(); i++) {
          std::get<1>(fields[i]) = std::stoi(sizes[i]);
        }
      } else if (token == "TYPE") {
        std::deque<std::string> types;
        boost::split(types, line, boost::is_any_of(" "));
        types.pop_front();

        if (fields.size() != types.size()) {
          fields.resize(types.size());
        }

        for (int i = 0; i < types.size(); i++) {
          std::get<2>(fields[i]) = types[i][0];
        }
      } else if (token == "WIDTH") {
        sst >> width;
      } else if (token == "HEIGHT") {
        sst >> height;
      } else if (token == "POINTS") {
        sst >> points;
      } else if (token == "DATA") {
        std::string data_type;
        sst >> data_type;
        ascii = (data_type == "ascii");
        return true;
      }
    }

    std::cerr << "error: PCD file parse error!!" << std::endl;
    return false;
  }

  double version;
  std::vector<std::tuple<std::string, int, char>> fields;
  int width;
  int height;
  int points;
  bool ascii;
};

std::shared_ptr<PCDData> read_pcd_binary(const PCDMetaData& metadata, std::istream& ist) {}

std::shared_ptr<PCDData> read_pcd_ascii(const PCDMetaData& metadata, std::istream& ist) {
  std::shared_ptr<PCDData> data(new PCDData);

  for (const auto& field : metadata.fields) {
    const auto& name = std::get<0>(field);

    if (name == "x" || name == "y" || name == "z") {
      data->vertices.resize(metadata.points);
    } else if (name == "intensity") {
      data->intensities.resize(metadata.points);
    } else if (name == "rgb") {
      data->colors.resize(metadata.points);
    } else if (name == "normal_x" || name == "normal_y" || name == "normal_z") {
      data->normals.resize(metadata.points);
    }
  }

  for (int i = 0; i < metadata.points; i++) {
    if (ist.eof()) {
      std::cerr << "error: PCD reader unexpectedly reached EOF!!" << std::endl;
      return nullptr;
    }

    std::string line;
    std::getline(ist, line);

    for (const auto& field : metadata.fields) {
    }
  }
}

std::shared_ptr<PCDData> load_pcd(const std::string& filename) {
  std::ifstream ifs(filename);
  if (!ifs) {
    std::cerr << "error: failed to open " << filename << std::endl;
    return nullptr;
  }

  PCDMetaData metadata;
  if (!metadata.read(ifs)) {
    return nullptr;
  }

  return nullptr;
}

}  // namespace glk
