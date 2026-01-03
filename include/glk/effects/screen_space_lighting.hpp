#ifndef GLK_SCREEN_SPACE_LIGHTING_HPP
#define GLK_SCREEN_SPACE_LIGHTING_HPP

#include <glk/effects/screen_effect.hpp>

namespace glk {

class ScreenSpaceSplatting;
class ScreenSpaceAttributeEstimation;

class ScreenSpaceLighting : public ScreenEffect {
public:
  enum class DIFFUSE_MODEL { ZERO, ONE, LAMBERT, DISNEY, NORMALIZED_DISNEY, OREN_NAYAR };
  enum class SPECULAR_MODEL { ZERO, PHONG, BLINN_PHONG, COOK_TORRANCE };
  enum class OCCLUSION_MODEL { ZERO, AMBIENT_OCCLUSION };
  enum class IRIDESCENCE_MODEL { ZERO, IRIDESCENCE1, IRIDESCENCE2, IRIDESCENCE3 };

  ScreenSpaceLighting(const Eigen::Vector2i& size = Eigen::Vector2i(1920, 1080), bool use_splatting = false);
  virtual ~ScreenSpaceLighting() override;

  const glk::Texture& position() const;
  const glk::Texture& normal() const;
  const glk::Texture& occlusion() const;

  void set_diffuse_model(DIFFUSE_MODEL model);
  void set_specular_model(SPECULAR_MODEL model);
  void set_occlusion_model(OCCLUSION_MODEL model);
  void set_iridescence_model(IRIDESCENCE_MODEL model);

  float get_albedo() const;
  float get_roughness() const;

  void set_albedo(float albedo);
  void set_roughness(float roughness);

  int num_lights() const;
  void set_light(int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color);
  void set_light(int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color, const Eigen::Vector2f& attenuation, float max_range);
  void set_directional_light(int i, const Eigen::Vector3f& direction, const Eigen::Vector4f& color);

  bool is_light_directional(int i) const;
  float get_light_range(int i) const;
  const Eigen::Vector2f& get_light_attenuation(int i) const;
  const Eigen::Vector3f& get_light_pos(int i) const;
  const Eigen::Vector3f& get_light_dir(int i) const;
  const Eigen::Vector4f& get_light_color(int i) const;

  void set_light_directional(int i, bool directional);
  void set_light_range(int i, float range);
  void set_light_attenuation(int i, const Eigen::Vector2f& attenuation);
  void set_light_pos(int i, const Eigen::Vector3f& pos);
  void set_light_dir(int i, const Eigen::Vector3f& dir);
  void set_light_color(int i, const Eigen::Vector4f& color);

  virtual void set_size(const Eigen::Vector2i& size) override;
  virtual void draw(
    const TextureRenderer& renderer,
    const glk::Texture& color_texture,
    const glk::Texture& depth_texture,
    const TextureRendererInput::Ptr& input,
    glk::FrameBuffer* frame_buffer = nullptr) override;

private:
  bool load_shader();

private:
  std::unique_ptr<glk::ScreenSpaceSplatting> splatting;
  std::unique_ptr<glk::ScreenSpaceAttributeEstimation> ssae;
  std::unique_ptr<glk::Texture> iridescence_texture;

  glk::GLSLShader lighting_shader;

  DIFFUSE_MODEL diffuse_model;
  SPECULAR_MODEL specular_model;
  OCCLUSION_MODEL occlusion_model;
  IRIDESCENCE_MODEL iridescence_model;

  float albedo;
  float roughness;

  bool light_updated;
  std::vector<int> light_directional;
  std::vector<float> light_range;
  std::vector<Eigen::Vector2f> light_attenuation;
  std::vector<Eigen::Vector3f> light_pos;
  std::vector<Eigen::Vector4f> light_color;
};

}  // namespace glk

#endif