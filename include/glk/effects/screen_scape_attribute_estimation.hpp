#ifndef GLK_SCREEN_SPACE_ATTRIBUTE_ESTIMATION_HPP
#define GLK_SCREEN_SPACE_ATTRIBUTE_ESTIMATION_HPP

#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

class ScreenSpaceAttributeEstimation : public ScreenEffect {
public:
  enum class BufferType { NONE, DEPTH, POSITION, SMOOTHED_POSITION_X, SMOOTHED_POSITION, NORMAL, SSAO, SMOOTHED_SSAO_X, SMOOTHED_SSAO };

  ScreenSpaceAttributeEstimation(const Eigen::Vector2i& size = Eigen::Vector2i(1920, 1080), BufferType rendering_type = BufferType::NONE);
  virtual ~ScreenSpaceAttributeEstimation() override;

  virtual void set_size(const Eigen::Vector2i& size) override;

  void set_smooth_normal(bool smooth_normal);
  void set_rendering_buffer(BufferType buffer_type);

  const glk::Texture& position() const;
  const glk::Texture& normal() const;
  const glk::Texture& occlusion() const;

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer = nullptr) override;

private:
  bool smooth_normal;
  BufferType rendering_type;

  glk::GLSLShader texture_shader;
  glk::GLSLShader pos_shader;
  glk::GLSLShader normal_shader;
  glk::GLSLShader occlusion_shader;
  glk::GLSLShader bilateral_shader;

  std::unique_ptr<glk::Texture> randomization_texture;

  std::unique_ptr<glk::FrameBuffer> position_buffer;
  std::unique_ptr<glk::FrameBuffer> position_smoothing_x_buffer;
  std::unique_ptr<glk::FrameBuffer> position_smoothing_y_buffer;

  std::unique_ptr<glk::FrameBuffer> normal_buffer;
  std::unique_ptr<glk::FrameBuffer> occlusion_buffer;
  std::unique_ptr<glk::FrameBuffer> bilateral_x_buffer;
  std::unique_ptr<glk::FrameBuffer> bilateral_y_buffer;
};

}  // namespace glk

#endif