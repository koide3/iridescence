#include <glk/experimental/aug_voxelmap.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

#include <glk/path.hpp>
#include <glk/io/image_io.hpp>

namespace glk {

// ============================================================================
// Constructor / Destructor
// ============================================================================

AugVoxelMap::AugVoxelMap(const std::string& asset_path, const std::string& voxel_info_path) {
  vao = vbo_position = vbo_type = 0;
  texture_array = 0;
  num_voxels = 0;
  num_layers = 0;

  // Parse voxel info CSV
  parse_voxel_info(voxel_info_path);

  // Load textures and create GL_TEXTURE_2D_ARRAY
  create_texture_array(asset_path);

  // Create shader pipeline
  shader = std::make_unique<glk::GLSLShader>();
  const auto data_path = glk::get_data_path();
  shader->attach_source(data_path + "/shader/voxelmap.vert", GL_VERTEX_SHADER);
  shader->attach_source(data_path + "/shader/voxelmap.geom", GL_GEOMETRY_SHADER);
  shader->attach_source(data_path + "/shader/voxelmap.frag", GL_FRAGMENT_SHADER);
  if (!shader->link_program()) {
    std::cerr << "Failed to link voxelmap shader program." << std::endl;
  }
}

AugVoxelMap::~AugVoxelMap() {
  if (vbo_type) glDeleteBuffers(1, &vbo_type);
  if (vbo_position) glDeleteBuffers(1, &vbo_position);
  if (vao) glDeleteVertexArrays(1, &vao);
  if (texture_array) glDeleteTextures(1, &texture_array);
}

// ============================================================================
// init_data
// ============================================================================

void AugVoxelMap::init_data(const Eigen::Vector4i* blocks, int num_blocks, double resolution) {
  this->resolution = static_cast<float>(resolution);
  this->num_voxels = num_blocks;

  const GLsizeiptr pos_buf_size = static_cast<GLsizeiptr>(sizeof(Eigen::Vector3f)) * num_blocks;
  const GLsizeiptr face_buf_size = static_cast<GLsizeiptr>(sizeof(Eigen::Vector3f)) * num_blocks;
  constexpr GLbitfield map_flags = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo_position);
  glGenBuffers(1, &vbo_type);

  // Allocate buffers
  glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
  glBufferData(GL_ARRAY_BUFFER, pos_buf_size, nullptr, GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_type);
  glBufferData(GL_ARRAY_BUFFER, face_buf_size, nullptr, GL_STATIC_DRAW);

  // Map position buffer
  glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
  auto* positions = static_cast<Eigen::Vector3f*>(glMapBufferRange(GL_ARRAY_BUFFER, 0, pos_buf_size, map_flags));

  // Map face-layer buffer (top, side, bottom layers per voxel)
  glBindBuffer(GL_ARRAY_BUFFER, vbo_type);
  auto* face_layers = static_cast<Eigen::Vector3f*>(glMapBufferRange(GL_ARRAY_BUFFER, 0, face_buf_size, map_flags));

  for (int i = 0; i < num_blocks; ++i) {
    positions[i] = (blocks[i].head<3>().cast<double>() * resolution).cast<float>();
    int type_id = blocks[i].w();
    auto it = face_layer_map.find(type_id);
    if (it != face_layer_map.end()) {
      face_layers[i] = it->second;
    } else {
      face_layers[i] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
  }

  // Unmap
  glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
  glUnmapBuffer(GL_ARRAY_BUFFER);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_type);
  glUnmapBuffer(GL_ARRAY_BUFFER);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// ============================================================================
// draw
// ============================================================================

void AugVoxelMap::draw(glk::GLSLShader& shader) const {
  // Read uniforms from the viewer's shader
  shader.use();
  const Eigen::Matrix4f model_matrix = shader.get_uniform_cache<Eigen::Matrix4f>(glk::hash("model_matrix"));
  const Eigen::Matrix4f view_matrix = shader.get_uniform_cache<Eigen::Matrix4f>(glk::hash("view_matrix"));
  const Eigen::Matrix4f projection_matrix = shader.get_uniform_cache<Eigen::Matrix4f>(glk::hash("projection_matrix"));

  const auto info_enabled = shader.get_uniform_cache_safe<int>(glk::hash("info_enabled"));
  const auto info_values = shader.get_uniform_cache_safe<Eigen::Vector4i>(glk::hash("info_values"));
  const auto normal_enabled = shader.get_uniform_cache_safe<int>(glk::hash("normal_enabled"));
  const auto partial_rendering_enabled = shader.get_uniform_cache_safe<int>(glk::hash("partial_rendering_enabled"));
  const auto dynamic_object = shader.get_uniform_cache_safe<int>(glk::hash("dynamic_object"));

  // Switch to our shader
  this->shader->use();

  this->shader->set_uniform(glk::hash("model_matrix"), model_matrix);
  this->shader->set_uniform(glk::hash("view_matrix"), view_matrix);
  this->shader->set_uniform(glk::hash("projection_matrix"), projection_matrix);
  this->shader->set_uniform(glk::hash("resolution"), resolution);

  // Forward G-buffer control uniforms
  if (info_enabled) this->shader->set_uniform(glk::hash("info_enabled"), *info_enabled);
  if (info_values) this->shader->set_uniform(glk::hash("info_values"), *info_values);
  if (normal_enabled) this->shader->set_uniform(glk::hash("normal_enabled"), *normal_enabled);
  if (partial_rendering_enabled) this->shader->set_uniform(glk::hash("partial_rendering_enabled"), *partial_rendering_enabled);
  if (dynamic_object) this->shader->set_uniform(glk::hash("dynamic_object"), *dynamic_object);

  // Bind texture array to unit 0
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_ARRAY, texture_array);
  this->shader->set_uniform(glk::hash("texture_array"), 0);

  // Set up vertex attributes
  glBindVertexArray(vao);

  GLint pos_loc = this->shader->attrib("vert_position");
  glEnableVertexAttribArray(pos_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_position);
  glVertexAttribPointer(pos_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  GLint face_loc = this->shader->attrib("vert_face_layers");
  glEnableVertexAttribArray(face_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_type);
  glVertexAttribPointer(face_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glDrawArrays(GL_POINTS, 0, num_voxels);

  glDisableVertexAttribArray(pos_loc);
  glDisableVertexAttribArray(face_loc);
  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

  // Restore viewer's shader
  shader.use();
}

// ============================================================================
// parse_voxel_info
// ============================================================================

void AugVoxelMap::parse_voxel_info(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << "Failed to open voxel info: " << path << std::endl;
    return;
  }

  std::string line;
  std::getline(ifs, line);  // skip header

  while (std::getline(ifs, line)) {
    std::istringstream ss(line);
    std::string id_str, name, count_str;
    std::getline(ss, id_str, ',');
    std::getline(ss, name, ',');
    std::getline(ss, count_str, ',');

    int id = std::stoi(id_str);

    // Strip "minecraft:" prefix
    if (name.find("minecraft:") == 0) {
      name = name.substr(10);
    }

    type_names[id] = name;
  }

  std::cout << "Parsed " << type_names.size() << " voxel types from CSV." << std::endl;
}

// ============================================================================
// Texture helpers
// ============================================================================

bool AugVoxelMap::texture_file_exists(const std::string& path) {
  std::ifstream test(path);
  return test.good();
}

std::string AugVoxelMap::try_texture_with_suffixes(const std::string& base_dir, const std::string& name) {
  static const std::vector<std::string> suffixes = {"", "_side", "_still", "_front", "_top"};
  for (const auto& suffix : suffixes) {
    std::string path = base_dir + name + suffix + ".png";
    if (texture_file_exists(path)) return path;
  }
  // Also try plural form (e.g., "mud_brick" -> "mud_bricks", "stone_brick" -> "stone_bricks")
  {
    std::string path = base_dir + name + "s.png";
    if (texture_file_exists(path)) return path;
  }
  return "";
}

std::string AugVoxelMap::find_texture_path(const std::string& asset_path, const std::string& block_name) {
  const std::string base = asset_path + "/textures/block/";

  // Explicit overrides for blocks with non-obvious texture names
  static const std::unordered_map<std::string, std::string> explicit_map = {
    {"wall_torch", "torch"},
    {"soul_wall_torch", "soul_torch"},
    {"redstone_wall_torch", "redstone_torch"},
    {"bubble_column", "water_still"},
    {"water_cauldron", "cauldron_side"},
    {"lava_cauldron", "cauldron_side"},
    {"powder_snow_cauldron", "cauldron_side"},
    {"moss_carpet", "moss_block"},
    {"campfire", "campfire_log_lit"},
    {"soul_campfire", "soul_campfire_log_lit"},
    {"wheat", "wheat_stage7"},
    {"potatoes", "wheat_stage7"},
    {"carrots", "wheat_stage7"},
    {"sweet_berry_bush", "sweet_berry_bush_stage3"},
    {"fire", "campfire_fire"},
    {"soul_fire", "soul_campfire_fire"},
    {"chest", "oak_planks"},
    {"trapped_chest", "oak_planks"},
    {"barrel", "barrel_side"},
    {"trial_spawner", "spawner"},
    {"decorated_pot", "terracotta"},
    {"suspicious_gravel", "gravel"},
    {"suspicious_sand", "sand"},
    {"mangrove_wood", "mangrove_log"},
    // Two-block-tall plants: use the bottom half texture
    {"tall_grass", "tall_grass_bottom"},
    {"large_fern", "large_fern_bottom"},
    {"peony", "peony_bottom"},
    {"rose_bush", "rose_bush_bottom"},
    {"sunflower", "sunflower_front"},
    {"tall_seagrass", "tall_seagrass_bottom"},
  };

  auto it = explicit_map.find(block_name);
  if (it != explicit_map.end()) {
    std::string result = try_texture_with_suffixes(base, it->second);
    if (!result.empty()) return result;
  }

  // Also handle "potted_X" -> try X
  std::string name = block_name;
  if (name.find("potted_") == 0) {
    name = name.substr(7);  // strip "potted_"
  }

  // 1. Try exact block name
  std::string result = try_texture_with_suffixes(base, name);
  if (!result.empty()) return result;

  // 2. Strip "waxed_" prefix (waxed_oxidized_copper -> oxidized_copper)
  if (name.find("waxed_") == 0) {
    name = name.substr(6);
    result = try_texture_with_suffixes(base, name);
    if (!result.empty()) return result;
  }

  // 3. Strip "infested_" prefix (infested_deepslate -> deepslate)
  if (name.find("infested_") == 0) {
    name = name.substr(9);
    result = try_texture_with_suffixes(base, name);
    if (!result.empty()) return result;
  }

  // 4. Strip "_block" suffix (magma_block -> magma)
  if (name.size() > 6 && name.substr(name.size() - 6) == "_block") {
    result = try_texture_with_suffixes(base, name.substr(0, name.size() - 6));
    if (!result.empty()) return result;
  }

  // 5. Handle "_wood" suffix (mangrove_wood -> mangrove_log)
  if (name.size() > 5 && name.substr(name.size() - 5) == "_wood") {
    result = try_texture_with_suffixes(base, name.substr(0, name.size() - 5) + "_log");
    if (!result.empty()) return result;
  }

  // 5. Handle compound block suffixes (stairs, slab, wall, fence, etc.)
  //    These blocks share textures with their base material
  static const std::vector<std::string> compound_suffixes = {
    "_stairs",
    "_slab",
    "_wall",
    "_fence_gate",
    "_fence",
    "_button",
    "_pressure_plate",
    "_door",
    "_wall_sign",
    "_sign",
    "_carpet",
  };

  for (const auto& suffix : compound_suffixes) {
    if (name.size() > suffix.size() && name.substr(name.size() - suffix.size()) == suffix) {
      std::string base_material = name.substr(0, name.size() - suffix.size());

      // Try the base material directly
      result = try_texture_with_suffixes(base, base_material);
      if (!result.empty()) return result;

      // Wood types use "_planks" texture for structural variants
      static const std::vector<std::string> wood_types = {
        "oak",
        "spruce",
        "birch",
        "jungle",
        "dark_oak",
        "acacia",
        "mangrove",
        "cherry",
        "bamboo",
        "crimson",
        "warped",
        "pale_oak",
      };
      for (const auto& wood : wood_types) {
        if (base_material == wood) {
          result = try_texture_with_suffixes(base, wood + "_planks");
          if (!result.empty()) return result;
        }
      }

      // "stone" base -> try "stone" and "cobblestone"
      if (base_material == "stone") {
        result = try_texture_with_suffixes(base, "stone");
        if (!result.empty()) return result;
      }

      break;  // Only strip one compound suffix
    }
  }

  // 6. Handle beds — use wool texture of matching color
  if (name.size() > 4 && name.substr(name.size() - 4) == "_bed") {
    std::string color = name.substr(0, name.size() - 4);
    result = try_texture_with_suffixes(base, color + "_wool");
    if (!result.empty()) return result;
  }

  // 7. Handle "_down_base" / "_up_base" suffixed blocks (pointed_dripstone)
  result = try_texture_with_suffixes(base, name + "_down_base");
  if (!result.empty()) return result;
  result = try_texture_with_suffixes(base, name + "_up_base");
  if (!result.empty()) return result;

  // 8. Try the block name with "_block" appended (moss_carpet -> moss_block)
  result = try_texture_with_suffixes(base, name + "_block");
  if (!result.empty()) return result;

  return "";
}

// ============================================================================
// Tinting
// ============================================================================

AugVoxelMap::TintColor AugVoxelMap::sample_colormap(const std::string& asset_path, const std::string& colormap_name) {
  const std::string path = asset_path + "/textures/colormap/" + colormap_name + ".png";
  int w, h;
  std::vector<unsigned char> px;
  if (glk::load_image(path, w, h, px) && w >= 128 && h >= 128) {
    int x = w / 2, y = h / 2;
    int idx = (y * w + x) * 4;
    return {px[idx], px[idx + 1], px[idx + 2]};
  }
  return {86, 168, 60};
}

AugVoxelMap::TintType AugVoxelMap::get_tint_type(const std::string& block_name) {
  static const std::vector<std::string> foliage_blocks = {
    "oak_leaves",
    "dark_oak_leaves",
    "jungle_leaves",
    "acacia_leaves",
    "mangrove_leaves",
    "vine",
    "pale_oak_leaves",
    "leaf_litter",
  };
  static const std::vector<std::string> grass_blocks = {
    "short_grass",
    "tall_grass",
    "fern",
    "large_fern",
    "sugar_cane",
    "moss_block",
    "moss_carpet",
  };
  for (const auto& b : foliage_blocks) {
    if (block_name == b) return TintType::Foliage;
  }
  for (const auto& b : grass_blocks) {
    if (block_name == b) return TintType::Grass;
  }
  if (block_name == "spruce_leaves") return TintType::Spruce;
  if (block_name == "birch_leaves") return TintType::Birch;
  if (block_name == "water" || block_name == "bubble_column") return TintType::Water;
  return TintType::None;
}

bool AugVoxelMap::is_cross_block(const std::string& name) {
  static const std::vector<std::string> cross_blocks = {
    // Flowers
    "dandelion",
    "poppy",
    "blue_orchid",
    "allium",
    "azure_bluet",
    "red_tulip",
    "orange_tulip",
    "white_tulip",
    "pink_tulip",
    "oxeye_daisy",
    "cornflower",
    "lily_of_the_valley",
    "wither_rose",
    "torchflower",
    "open_eyeblossom",
    "closed_eyeblossom",
    "wildflowers",
    "peony",
    "rose_bush",
    // Grasses & ferns
    "short_grass",
    "tall_grass",
    "fern",
    "large_fern",
    "dead_bush",
    "bush",
    "firefly_bush",
    "seagrass",
    "tall_seagrass",
    // Saplings
    "oak_sapling",
    "spruce_sapling",
    "birch_sapling",
    "jungle_sapling",
    "dark_oak_sapling",
    "acacia_sapling",
    "cherry_sapling",
    "mangrove_propagule",
    "pale_oak_sapling",
    // Mushrooms
    "brown_mushroom",
    "red_mushroom",
    // Nether vegetation
    "crimson_roots",
    "warped_roots",
    "nether_sprouts",
    "crimson_fungus",
    "warped_fungus",
    // Aquatic / cave vegetation
    "kelp",
    "kelp_plant",
    "cave_vines",
    "cave_vines_plant",
    "lily_pad",
    // Other cross-model blocks
    "sweet_berry_bush",
    "hanging_roots",
    "cobweb",
    "leaf_litter",
    "torch",
    "wall_torch",
    "soul_torch",
    "soul_wall_torch",
    "redstone_torch",
    "redstone_wall_torch",
  };
  for (const auto& b : cross_blocks) {
    if (name == b) return true;
  }
  if (name.find("potted_") == 0) return true;
  return false;
}

void AugVoxelMap::apply_tint(std::vector<unsigned char>& pixels, int width, int height, TintColor tint) {
  for (int i = 0; i < width * height; i++) {
    int idx = i * 4;
    pixels[idx + 0] = static_cast<unsigned char>(pixels[idx + 0] * tint.r / 255);
    pixels[idx + 1] = static_cast<unsigned char>(pixels[idx + 1] * tint.g / 255);
    pixels[idx + 2] = static_cast<unsigned char>(pixels[idx + 2] * tint.b / 255);
  }
}

void AugVoxelMap::generate_missing_texture(std::vector<unsigned char>& pixels) {
  pixels.resize(16 * 16 * 4);
  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      int idx = (y * 16 + x) * 4;
      bool checker = ((x / 4) + (y / 4)) % 2 == 0;
      pixels[idx + 0] = checker ? 255 : 0;  // R (magenta/black checkerboard)
      pixels[idx + 1] = 0;
      pixels[idx + 2] = checker ? 255 : 0;  // B
      pixels[idx + 3] = 255;
    }
  }
}

// ============================================================================
// Face texture resolution
// ============================================================================

AugVoxelMap::FaceTexInfo AugVoxelMap::resolve_face_textures(const std::string& asset_path, const std::string& block_name) const {
  const std::string base = asset_path + "/textures/block/";
  std::string default_path = find_texture_path(asset_path, block_name);
  TintType default_tint = get_tint_type(block_name);

  FaceTexInfo info;
  info.top_path = info.side_path = info.bottom_path = default_path;
  info.top_tint = info.side_tint = info.bottom_tint = default_tint;

  // Cross blocks (flowers, grasses, etc.) use a single texture — skip cube face resolution
  if (is_cross_block(block_name)) {
    return info;
  }

  // For grass_block: top is tinted grayscale, side has built-in color, bottom is dirt
  if (block_name == "grass_block") {
    if (texture_file_exists(base + "grass_block_top.png")) info.top_path = base + "grass_block_top.png";
    info.top_tint = TintType::Grass;
    if (texture_file_exists(base + "grass_block_side.png")) info.side_path = base + "grass_block_side.png";
    info.side_tint = TintType::None;
    if (texture_file_exists(base + "dirt.png")) info.bottom_path = base + "dirt.png";
    info.bottom_tint = TintType::None;
    return info;
  }

  // For mycelium/podzol/dirt_path: top/side differ, bottom is dirt
  for (const auto& name : {"mycelium", "podzol"}) {
    if (block_name == name) {
      if (texture_file_exists(base + block_name + "_top.png")) info.top_path = base + block_name + "_top.png";
      if (texture_file_exists(base + block_name + "_side.png")) info.side_path = base + block_name + "_side.png";
      if (texture_file_exists(base + "dirt.png")) info.bottom_path = base + "dirt.png";
      info.top_tint = info.side_tint = info.bottom_tint = TintType::None;
      return info;
    }
  }
  if (block_name == "dirt_path") {
    if (texture_file_exists(base + "dirt_path_top.png")) info.top_path = base + "dirt_path_top.png";
    if (texture_file_exists(base + "dirt_path_side.png")) info.side_path = base + "dirt_path_side.png";
    if (texture_file_exists(base + "dirt.png")) info.bottom_path = base + "dirt.png";
    info.top_tint = info.side_tint = info.bottom_tint = TintType::None;
    return info;
  }

  // For blocks with _top variant (logs, sandstone, deepslate, pillars, etc.)
  // Try: top/bottom = X_top, side = X (or X_side)
  std::string stripped = block_name;  // base name for texture lookup
  // Strip prefixes that were applied during find_texture_path
  if (stripped.find("waxed_") == 0) stripped = stripped.substr(6);
  if (stripped.find("infested_") == 0) stripped = stripped.substr(9);

  std::string top_candidate = base + stripped + "_top.png";
  if (texture_file_exists(top_candidate)) {
    info.top_path = top_candidate;
    info.bottom_path = top_candidate;  // Default: bottom same as top
    info.top_tint = info.bottom_tint = default_tint;

    // Check for explicit bottom texture (sandstone_bottom etc.)
    std::string bottom_candidate = base + stripped + "_bottom.png";
    if (texture_file_exists(bottom_candidate)) {
      info.bottom_path = bottom_candidate;
    }

    // Side: prefer X.png, then X_side.png
    if (texture_file_exists(base + stripped + ".png")) {
      info.side_path = base + stripped + ".png";
    } else if (texture_file_exists(base + stripped + "_side.png")) {
      info.side_path = base + stripped + "_side.png";
    }
  }

  return info;
}

// ============================================================================
// Texture loading & processing
// ============================================================================

bool AugVoxelMap::load_and_process_texture(const std::string& path, TintType tint, const TintColors& tints, std::vector<unsigned char>& pixels) {
  int width, height;
  if (!glk::load_image(path, width, height, pixels)) {
    return false;
  }

  // Crop to 16x16 if needed (handles animated spritesheets like water_still.png)
  if (width != 16 || height != 16) {
    if (width >= 16 && height >= 16) {
      std::vector<unsigned char> cropped(16 * 16 * 4);
      for (int y = 0; y < 16; y++) {
        std::memcpy(cropped.data() + y * 16 * 4, pixels.data() + y * width * 4, 16 * 4);
      }
      pixels = std::move(cropped);
      width = height = 16;
    } else {
      return false;
    }
  }

  // Apply biome tint
  switch (tint) {
    case TintType::Foliage:
      apply_tint(pixels, width, height, tints.foliage);
      break;
    case TintType::Grass:
      apply_tint(pixels, width, height, tints.grass);
      break;
    case TintType::Spruce:
      apply_tint(pixels, width, height, tints.spruce);
      break;
    case TintType::Birch:
      apply_tint(pixels, width, height, tints.birch);
      break;
    case TintType::Water:
      apply_tint(pixels, width, height, tints.water);
      break;
    default:
      break;
  }
  return true;
}

// ============================================================================
// create_texture_array
// ============================================================================

void AugVoxelMap::create_texture_array(const std::string& asset_path) {
  // Sample biome tint colors from colormaps
  TintColors tints;
  tints.foliage = sample_colormap(asset_path, "foliage");
  tints.grass = sample_colormap(asset_path, "grass");
  tints.spruce = {97, 153, 97};
  tints.birch = {128, 167, 85};
  tints.water = {63, 118, 228};

  std::cout << "Foliage tint: (" << (int)tints.foliage.r << ", " << (int)tints.foliage.g << ", " << (int)tints.foliage.b << ")" << std::endl;
  std::cout << "Grass tint: (" << (int)tints.grass.r << ", " << (int)tints.grass.g << ", " << (int)tints.grass.b << ")" << std::endl;

  // Collect all unique (path, tint) pairs and assign texture array layers
  std::map<std::pair<std::string, TintType>, int> tex_to_layer;
  int next_layer = 0;

  // Resolve face textures for each type
  struct TypeFaces {
    FaceTexInfo info;
    int top_layer, side_layer, bottom_layer;
  };
  std::unordered_map<int, TypeFaces> type_faces;

  auto alloc_layer = [&](const std::string& path, TintType tint) -> int {
    if (path.empty()) return -1;
    auto key = std::make_pair(path, tint);
    auto it = tex_to_layer.find(key);
    if (it != tex_to_layer.end()) return it->second;
    int layer = next_layer++;
    tex_to_layer[key] = layer;
    return layer;
  };

  for (const auto& [id, name] : type_names) {
    FaceTexInfo fi = resolve_face_textures(asset_path, name);
    int top_l = alloc_layer(fi.top_path, fi.top_tint);
    int side_l = alloc_layer(fi.side_path, fi.side_tint);
    int bottom_l = alloc_layer(fi.bottom_path, fi.bottom_tint);
    type_faces[id] = {fi, top_l, side_l, bottom_l};
  }

  num_layers = next_layer;
  if (num_layers == 0) num_layers = 1;

  std::cout << "Allocated " << num_layers << " unique texture layers." << std::endl;

  // Create the GL_TEXTURE_2D_ARRAY
  glGenTextures(1, &texture_array);
  glBindTexture(GL_TEXTURE_2D_ARRAY, texture_array);
  glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8, 16, 16, num_layers, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

  // Fill all layers with the missing-texture pattern
  std::vector<unsigned char> missing_pixels;
  generate_missing_texture(missing_pixels);
  for (int i = 0; i < num_layers; i++) {
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, 16, 16, 1, GL_RGBA, GL_UNSIGNED_BYTE, missing_pixels.data());
  }

  // Load each unique texture into its assigned layer
  int loaded = 0, failed = 0;
  for (const auto& [key, layer] : tex_to_layer) {
    const auto& [path, tint] = key;
    std::vector<unsigned char> pixels;
    if (!load_and_process_texture(path, tint, tints, pixels)) {
      std::cerr << "Failed to load: " << path << std::endl;
      failed++;
      continue;
    }
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, layer, 16, 16, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    loaded++;
  }

  std::cout << "Texture array: " << loaded << " loaded, " << failed << " failed (total layers: " << num_layers << ")" << std::endl;

  // Build face_layer_map for init_data to look up
  for (const auto& [id, tf] : type_faces) {
    const std::string& name = type_names.at(id);
    if (is_cross_block(name)) {
      // Cross blocks: encode as (-1, texture_layer, 0) — sentinel for geometry shader
      int layer = (tf.side_layer >= 0) ? tf.side_layer : ((tf.top_layer >= 0) ? tf.top_layer : 0);
      face_layer_map[id] = Eigen::Vector3f(-1.0f, static_cast<float>(layer), 0.0f);
    } else {
      int top = (tf.top_layer >= 0) ? tf.top_layer : 0;
      int side = (tf.side_layer >= 0) ? tf.side_layer : 0;
      int bottom = (tf.bottom_layer >= 0) ? tf.bottom_layer : 0;
      face_layer_map[id] = Eigen::Vector3f(static_cast<float>(top), static_cast<float>(side), static_cast<float>(bottom));
    }
  }

  // Use GL_NEAREST for pixel-art look
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
}

}  // namespace glk
