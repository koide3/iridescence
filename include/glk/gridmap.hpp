#ifndef GLK_GRIDMAP_HPP
#define GLK_GRIDMAP_HPP

#include <memory>
#include <glk/drawable.hpp>

namespace glk {

class Texture;

class GridMap : public Drawable {
public:
  enum class ColorMode { RAW = 0, TURBO, PROB, PROB_TURBO, RGBA };

  GridMap(double resolution, int width, int height, const unsigned char* values, int alpha = 255, ColorMode mode = ColorMode::PROB);
  GridMap(double resolution, int width, int height, float scale, const float* values, float alpha = 1.0f, ColorMode mode = ColorMode::PROB);
  virtual ~GridMap();

  virtual void draw(glk::GLSLShader& shader) const override;

  void update_color(int width, int height, const unsigned char* values, int alpha = 255, ColorMode mode = ColorMode::PROB);
  void update_color(const unsigned char* values, int alpha = 255, ColorMode mode = ColorMode::PROB);
  void update_color(int width, int height, float scale, const float* values, float alpha = 1.0f, ColorMode mode = ColorMode::PROB);
  void update_color(float scale, const float* values, float alpha = 1.0f, ColorMode mode = ColorMode::PROB);

private:
  GridMap(const GridMap&);
  GridMap& operator=(const GridMap&);

  void init_vao(double resolution, int width, int height);

private:
  GLuint vao;
  GLuint vbo;
  GLuint tbo;
  std::unique_ptr<Texture> texture;
};

}  // namespace glk

#endif