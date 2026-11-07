#ifndef GLK_GRIDMAP_HPP
#define GLK_GRIDMAP_HPP

#include <memory>
#include <glk/drawable.hpp>

namespace glk {

class Texture;

/**
 * @brief GridMap is a drawable that represents a 2D grid map with color values.
 *        It assumes that pixel values are stored in row-major order (values[y * width + x]),
 *        starting from the (-x, -y) corner (i.e., the first row lies at -y).
 *        The grid map lies on the XY plane, and the center is at the origin (0, 0, 0),
 *        thus it extends from (-resolution * width / 2, -resolution * height / 2) to (resolution * width / 2, resolution * height / 2).
 *        Use guik::TextureColor as shader setting.
 */
class GridMap : public Drawable {
public:
  /// @brief ColorMode defines how the color values are interpreted.
  /// @details Assuming N=width*height
  /// For uint8 values:
  ///        RAW: expects |values|=N, and rgb[i] = values[i]
  ///        TURBO: expects |values|=N, and rgb[i] = colormap(TURBO, values[i])
  ///        PROB: expects |values|=N, and rgb[i] = 100 - values[i]
  ///        PROB_TURBO: expects |values|=N, and rgb[i] = colormap(TURBO, (100 - values[i]) * 255.0 / 100.0)
  ///        RGBA: expects |values|=4*N, and rgba[i] = values[i*4:i*4+4]
  /// For float values:
  ///        RAW: expects |values|=N, and rgb[i] = scale * values[i]
  ///        TURBO: expects |values|=N, and rgb[i] = colormapf(TURBO, scale * values[i])
  ///        PROB: expects |values|=N, and rgb[i] = 1.0 - scale * values[i]
  ///        PROB_TURBO: expects |values|=N, and rgb[i] = colormapf(TURBO, 1.0 - scale * values[i])
  ///        RGBA: expects |values|=4*N, and rgba[i] = values[i*4:i*4+4] (scale is not applied)
  enum class ColorMode { RAW = 0, TURBO, PROB, PROB_TURBO, RGBA };

  /// @brief Constructor
  /// @param resolution  Resolution of each grid cell in world units.
  /// @param width       Width [cells]
  /// @param height      Height [cells]
  /// @param values      Pointer to the color values.
  /// @param alpha       Alpha value for the entire map. Ignored if ColorMode is RGBA.
  /// @param mode        Color mode for interpreting the values.
  GridMap(double resolution, int width, int height, const unsigned char* values, int alpha = 255, ColorMode mode = ColorMode::PROB);

  /// @brief Constructor
  /// @param resolution  Resolution of each grid cell in world units.
  /// @param width       Width [cells]
  /// @param height      Height [cells]
  /// @param scale       Scale factor for the color values. Ignored if ColorMode is RGBA.
  /// @param values      Pointer to the color values.
  /// @param alpha       Alpha value for the entire map. Ignored if ColorMode is RGBA.
  /// @param mode        Color mode for interpreting the values.
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