# Shader setting

## Shader setting

**guik::ShaderSetting** class holds rendering parameters of a drawable object (e.g., color mode and model matrix) to control the rendering process of the assigned object.

```cpp
// A sphere drawable object
std::shared_ptr<glk::Drawable> drawable = glk::Primitives::sphere();

// guik::ShaderSetting holds rendering parameters
int color_mode = glk::ColorMode::RAINBOW;
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
auto shader_setting = guik::ShaderSetting(color_mode, transformation)

// Register the pair of drawable object and shader setting to the viewer
viewer->update_drawable("drawable_name", drawable, shader_setting);
```

## Transformation

**guik::ShaderSetting** accepts Eigen transformations (e.g., ```Eigen::Matrix4f```, ```Eigen::Isometry3f```, ```Eigen::Affine3f``` and their double counterparts) as the model transformation.

It also has several utility methods to manipulate the model matrix. The utility methods applies a transformation on the right side of the original model transformation.
```cpp
// The model matrix becomes Identity() * Rotation(3.14rad, (1,0,0)) * Translation(1,0,0) * Scale(0.1)
viewer->update_drawable("drawable_name", drawable,
  guik::Rainbow().rotate(3.14f, {1.0f, 0.0f, 0.0f}).translate({1.0f, 0.0f, 0.0f}).scale(0.1f)
);
```

## Coloring schemes

There are four coloring schemes in Iridescence, and they have corresponding utility classes that are derived from ```guik::ShaderSetting```:

- **RAINBOW (guik::Rainbow)** scheme draws pixels with colors that encode the 3D position of each pixel (By default, it encodes the height (z) position of each pixel).
- **FLAT_COLOR (guik::FlatColor)** scheme draws pixels with a flat color.
- **VERTEX_COLOR (guik::VertexColor)** scheme draws pixels with interpolated colors of corresponding vertices.
- **TEXTURE_COLOR (guik::TextureColor)** scheme samples pixel colors from a texture.
- **VERTEX_COLORMAP (guik::VertexColorMap)** scheme draws pixels with colors that encode vertex colormap attribute (Similar to RAINBOW, but uses an arbitrary vertex scalar attribute like vertex intensity).

![Screenshot_20230101_004203](https://user-images.githubusercontent.com/31344317/210148371-c12e7126-2dc2-48e5-b43b-b57a7be9d92e.png)
Left to right: Rainbow, FlatColor, VertexColor, TextureColor (transparent)


```cpp
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

// RAINBOW
auto shader_setting = guik::Rainbow(transformation);

// FLAT_COLOR
Eigen::Vector4f color(1.0f, 0.5f, 0.0f, 1.0f);
auto shader_setting = guik::FlatColor(color, transformation);

// There are several flat color utility classes correspond to primitive colors
// guik::FlatRed() == guik::FlatColor({1.0f, 0.0f, 0.0f, 1.0f});
// guik::FlatGreen() == guik::FlatColor({0.0f, 1.0f, 0.0f, 1.0f});
// guik::FlatBlue() == guik::FlatColor({0.0f, 0.0f, 1.0f, 1.0f});
// guik::FlatOrange() == guik::FlatColor({1.0f, 0.5f, 0.0f, 1.0f});

// VERTEX_COLOR
auto shader_setting = guik::VertexColor(transformation);

// TEXTURE_COLOR with transparency
auto shader_setting = guik::TextureColor(transformation).make_transparent();

// VERTEX_COLORMAP
auto shader_setting = guik::VertexColorMap(transformation);
```

![Screenshot_20230101_005425](https://user-images.githubusercontent.com/31344317/210149282-38377bad-dfb8-4f86-a907-60cdcef10b92.png)
Example of point clouds rendered using the rainbow coloring scheme
