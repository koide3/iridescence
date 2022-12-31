# Shader setting

## Shader setting

**guik::ShaderSetting** class holds rendering parameters (e.g., color mode and model matrix) to control the rendering process of an assigned drawable.

```cpp
std::shared_ptr<glk::Drawable> drawable = glk::Primitives::sphere();

int color_mode = glk::ColorMode::RAINBOW;
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
auto shader_setting = glk::ShaderSetting(color_mode, transformation)

viewer->update_drawable("drawable_name", drawable, shader_setting);
```

## Coloring schemes

There are four coloring schemes and their corresponding utility classes that are derived from ```guik::ShaderSetting```:

- **RAINBOW (guik::Rainbow)** scheme draws pixels with colors that encode the 3D position of each pixel.
- **FLAT_COLOR (guik::FlatColor)** scheme draws pixels with a flat color.
- **VERTEX_COLOR (guik::VertexColor)** scheme draws pixels with colors of corresponding vertices.
- **TEXTURE_COLOR (guik::TextureColor)** scheme samples pixel colors from a texture.

```cpp
// RAINBOW
auto shader_setting = guik::Rainbow(transformation);

// FLAT_COLOR
Eigen::Vector4f color(1.0f, 0.5f, 0.0f, 1.0f);
auto shader_setting = guik::FlatColor(color, transformation);

// Flat color utility classes correspond to primitive colors
// guik::FlatRed() == guik::FlatColor({1.0f, 0.0f, 0.0f, 1.0f});
// guik::FlatGreen() == guik::FlatColor({0.0f, 1.0f, 0.0f, 1.0f});
// guik::FlatBlue() == guik::FlatColor({0.0f, 0.0f, 1.0f, 1.0f});
// guik::FlatOrange() == guik::FlatColor({1.0f, 0.5f, 0.0f, 1.0f});

// VERTEX_COLOR
auto shader_setting = guik::VertexColor(transformation);

// TEXTURE_COLOR
auto shader_setting = guik::TextureColor(transformation);
```


## Transformation

```Eigen::Matrix4f```, ```Eigen::Isometry3f```, ```Eigen::Affine3f``` and their double counterparts.

```guik::ShaderSetting``` has several utility methods to manipulate the model matrix:
```cpp
viewer->update_drawable("drawable_name", drawable,
  guik::Rainbow().translate({1.0f, 0.0f, 0.0f}).scale(0.1f)
);
```
