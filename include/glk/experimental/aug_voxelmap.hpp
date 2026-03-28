#ifndef GLK_EXPERIMENTAL_AUG_VOXELMAP_HPP
#define GLK_EXPERIMENTAL_AUG_VOXELMAP_HPP

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <GL/gl3w.h>
#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

class AugVoxelMap : public Drawable {
public:
  AugVoxelMap(const std::string& asset_path, const std::string& voxel_info_path);
  ~AugVoxelMap();

  void init_data(const Eigen::Vector4i* blocks, int num_blocks, double resolution);
  void draw(glk::GLSLShader& shader) const override;

private:
  // Minecraft stores foliage/grass textures as grayscale masks that get
  // multiplied by a biome-dependent tint at runtime.
  struct TintColor {
    unsigned char r, g, b;
  };

  enum class TintType { None, Foliage, Grass, Spruce, Birch, Water };

  // Per-face texture info for a block type
  struct FaceTexInfo {
    std::string top_path, side_path, bottom_path;
    TintType top_tint, side_tint, bottom_tint;
  };

  struct TintColors {
    TintColor foliage, grass, spruce, birch, water;
  };

  void parse_voxel_info(const std::string& path);
  void create_texture_array(const std::string& asset_path);

  static bool texture_file_exists(const std::string& path);
  static std::string try_texture_with_suffixes(const std::string& base_dir, const std::string& name);
  static std::string find_texture_path(const std::string& asset_path, const std::string& block_name);
  static TintColor sample_colormap(const std::string& asset_path, const std::string& colormap_name);
  static TintType get_tint_type(const std::string& block_name);
  static bool is_cross_block(const std::string& name);
  static void apply_tint(std::vector<unsigned char>& pixels, int width, int height, TintColor tint);
  static void generate_missing_texture(std::vector<unsigned char>& pixels);

  FaceTexInfo resolve_face_textures(const std::string& asset_path, const std::string& block_name) const;
  bool load_and_process_texture(const std::string& path, TintType tint, const TintColors& tints, std::vector<unsigned char>& pixels);

  std::unique_ptr<glk::GLSLShader> shader;
  GLuint texture_array;
  GLuint vao;
  GLuint vbo_position;
  GLuint vbo_type;
  float resolution;
  int num_voxels;
  int num_layers;
  std::unordered_map<int, std::string> type_names;
  std::unordered_map<int, Eigen::Vector3f> face_layer_map;  // type_id -> (top, side, bottom) layers
};

}  // namespace glk

#endif  // GLK_EXPERIMENTAL_AUG_VOXELMAP_HPP
