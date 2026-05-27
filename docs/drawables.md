# Drawables

Examples of lines, 3D primitives, and 2D drawings ([Code](https://github.com/koide3/iridescence/blob/master/src/example/02_light_viewer_primitives.cpp)):
![primitives](https://user-images.githubusercontent.com/31344317/210129208-2d126725-67d9-48ff-9eae-7af118a319f9.png)


## Shorthand methods


For frequently used drawable types, ```guik::LightViewer``` provides shorthand methods to quickly create and update drawables.

```cpp
// Primitives
viewer->update_sphere("sphere", guik::FlatRed());
viewer->update_wire_sphere("wire_sphere", guik::FlatRed());
viewer->update_coord("coord", guik::VertexColor());
viewer->update_wire_frustum("frustum", guik::FlatGreen());

// PointCloudBuffer
// Any of Vector(3|4)(f|d) are allowed as input
std::vector<Eigen::Vector4d> points = ...;
viewer->update_points("points", points, guik::Rainbow());

// NormalDistributions
std::vector<Eigen::Vector3f> means = ...;
std::vector<Eigen::Matrix3f> covs = ...;
float scale = 1.0f;
viewer->update_normal_dists("normal_dists", means, covs, scale, guik::Rainbow());

// ThinLine
std::vector<Eigen::Vector3f> line_vertices = ...;
bool line_strip = true;
viewer->update_thin_lines("lines", line_vertices, true, guik::FlatGreen());
```


## 3D Primitives

- Icosahedron
- Sphere
- Stanford bunny
- Cube
- Cone
- Coordinate system
- Frustum

```cpp
#include <glk/primitives/primitives.hpp>

// Solid and wire icosahedrons
glk::Primitives::icosahedron();
glk::Primitives::wire_icosahedron();

// Solid and wire spheres
glk::Primitives::sphere();
glk::Primitives::wire_sphere();

// Solid and wire bunnies
glk::Primitives::bunny();
glk::Primitives::wire_bunny();

// Solid and wire cubes
glk::Primitives::cube();
glk::Primitives::wire_cube();

// Solid and wire cones
glk::Primitives::cone();
glk::Primitives::wire_cone();

// RGB-colored coordinate systems rendered using GL_LINES and polygons
// They should be rendered with guik::VertexColor
glk::Primitives::coordinate_system();
glk::Primitives::solid_coordinate_system();

// Wire frustum for representing a camera pose (+Z=front)
glk::Primitives::wire_frustum();
```

![Screenshot_20221231_182844](https://user-images.githubusercontent.com/31344317/210131821-42071de7-3ace-433b-9cb4-d39d9444ee85.png)


## Lines

**glk::ThinLines** draws lines with GL_LINES. 
The thickness of lines is independent of the viewpoint.

```cpp
#include <glk/thin_lines.hpp>

// Line vertices
std::vector<Eigen::Vector3f> vertices = ...;

// If line_strip == true, lines are drawn between adjacent vertices (GL_LINE_STRIP).
// If line_strip == false, lines are drawn between vertices[i * 2] and vertices[i * 2 + 1] (GL_LINES).
bool line_strip = true;

// Create lines (All vertices are processed in order)
auto lines = std::make_shared<glk::ThinLines>(vertices, line_strip);

// Create lines with indexing
std::vector<unsigned int> indices = ...;
auto lines_with_indices = std::make_shared<glk::ThinLines>(vertices, indices, line_strip);

// Create lines with vertex colors
std::vector<Eigen::Vector4f> colors = ...;
auto lines_with_colors = std::make_shared<glk::ThinLines>(vertices, colors, line_strip);

// Set line width (glLineWidth)
lines->set_line_width(2.0f);
```


**glk::Lines** draws lines with polygons. The thickness of lines changes depending on the viewpoint and perspective.
```cpp
#include <glk/thin_lines.hpp>

float line_width = 0.1f;
std::vector<Eigen::Vector3f> vertices = ...;
std::vector<Eigen::Vector4f> colors = ...;
bool line_strip = true;

auto lines = std::make_shared<glk::Lines>(line_width, vertices, colors, line_strip);
```

![Screenshot_20221231_183450](https://user-images.githubusercontent.com/31344317/210131978-1b99b57e-193b-4196-8887-fffe51a858c6.png)


## Point cloud

**glk::PointCloudBuffer** holds and renders a 3D point cloud.

```cpp
#include <glk/pointcloud_buffer.hpp>

// Create PointCloudBuffer from std::vector<Eigen::Vector3f>
std::vector<Eigen::Vector3f> vertices = ...;
auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(vertices);

// Add vertex colors
std::vector<Eigen::Vector4f> colors = ...;
cloud_buffer->add_color(colors);

// Add vertex colors that encode scalar values in [0, 1]
std::vector<double> intensities = ...;
cloud_buffer->add_intensity(glk::COLORMAP::TURBO, intensities);

// Add vertex normals
std::vector<Eigen::Vector3f> normals = ...;
cloud_buffer->add_normals(normals);

// Add AUX point property
std::vector<float> values = ...;
int dim = 1;
cloud_buffer->add_buffer("radius", dim, values.data(), sizeof(float) * dim, values.size());

// Enlarge point size
auto shader_setting = guik::Rainbow().set_point_scale(2.0f);
viewer->update_drawable("points", cloud_buffer, shader_setting);
```

![Screenshot_20230101_005425](https://user-images.githubusercontent.com/31344317/210149282-38377bad-dfb8-4f86-a907-60cdcef10b92.png)
```glk::PointCloudBuffer``` rendered with ```guik::Rainbow```

**glk::IndexedPointCloudBuffer** enables specifying the indices of vertices to be rendered.

```cpp
#include <glk/indexed_pointcloud_buffer.hpp>

std::vector<Eigen::Vector3f> vertices = ...;
auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(vertices);

std::vector<unsigned int> indices = ...;
auto indexed_buffer = std::make_shared<glk::IndexedPointCloudBuffer>(cloud_buffer, indices);
```

### Point shape

The point shape (rectangles by default) can be changed to circles by setting `point_shape_mode=PointShapeMode::CIRCLE`.

```cpp
auto viewer = guik::viewer();
guik::ShaderSetting& global_setting = viewer->shader_setting();
global_setting.set_point_shape_mode(guik::PointShapeMode::RECTANGLE);  // Set default point shape mode to RECTANGLE. Alternatively, global_setting.set_point_shape_rectangle() can be used.
global_setting.set_point_shape_mode(guik::PointShapeMode::CIRCLE);     // Set default point shape mode to CIRCLE. Alternatively, global_setting.set_point_shape_circle() can be used.
```

### Point scale

**Screen space scaling (default)**  
The size of points is computed as `radius_pix = point_scale * point_size * nz + point_size_offset`, where `nz` is the fragment screen space depth in [0, 1]. By default `point_scale=1.0`, `point_size=10.0`, `point_size_offset=0.0`, and they can be updated by setting values to `guik::ShaderSetting`.

```cpp
auto viewer = guik::viewer();
guik::ShaderSetting& global_setting = viewer->shader_setting();
  global_setting.set_point_scale_screenspace();   // Set the point scale mode to screenspace
  global_setting.set_point_size(5.0f);            // Set the base point size to 5.0

auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(...);
// Make the size of points as twice large as the base point size
viewer->update_drawable("points", cloud_buffer, guik::FlatBlue().set_point_scale(2.0f));
```

**Metric space scaling**  
The size of points is computed based on the physical size specified as `radius_m = point_scale * point_size + point_size_offset`.

```cpp
auto viewer = guik::viewer();
guik::ShaderSetting& global_setting = viewer->shader_setting();
  global_setting.set_point_shape_circle();
  global_setting.set_point_scale_metric();
  global_setting.set_point_size(0.5f);  // Set default point radius to 0.5

auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(...);
// Set point radius to 0.5
viewer->update_drawable("points", cloud_buffer, guik::FlatBlue().set_point_size(0.5f));
```

![ss_1710604595 779500](https://github.com/koide3/iridescence/assets/31344317/f13f8c91-8dcc-4cc0-8dfa-9e4edcb2f518)
Red : Wire spheres (radius=0.5), Blue : Points rendered with PointCloudBuffer (`point_shape_mode=CIRCLE`, `point_scale_mode=METRIC`, `point_size=0.5`).


## Normal distributions

**glk::NormalDistributions**

```cpp
#include <glk/normal_distributions.hpp>

std::vector<Eigen::Vector3f> means = ...;
std::vector<Eigen::Matrix3f> covs = ...;
float scale = 1.0f;

auto normal_distributions = std::make_shared<glk::NormalDistributions>(means, covs, scale);
```

## Point splatting

**glk::Splatting**

```cpp
#include <glk/splatting.hpp>

// Create a PointCloudBuffer with normals
std::vector<Eigen::Vector3f> vertices = ...;
std::vector<Eigen::Vector3f> normals = ...;
auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(vertices);
cloud_buffer->add_normals(normals);

// Create a splatting shader
auto splatting_shader = glk::create_splatting_shader();

// Create a splatting instance
float point_radius = 0.1f;
auto splatting = std::make_shared<glk::Splatting>(splatting_shader);
splatting->set_point_radius(point_radius);
splatting->set_cloud_buffer(cloud_buffer);

// If vertex radius is enabled, the radius of each point is calculated as point_radius * vertex's radius.
// Otherwise, the fixed point_radius is used for rendering all points.
splatting->enable_vertex_radius();

std::vector<float> radii = ...;
cloud_buffer->add_buffer("radius", 1, radii.data(), sizeof(float), radii.size());
```

![points](https://user-images.githubusercontent.com/31344317/210149278-ac7a1424-5846-4a8c-94a8-dc2173229566.png)
Sparse point cloud

![splats](https://user-images.githubusercontent.com/31344317/210149280-7160f17f-c6cd-46c5-a66c-4a41d480db69.png)
Sparse point cloud rendered using ```glk::Splatting```

![splats2](https://user-images.githubusercontent.com/31344317/210149281-18ad2296-4bc6-4f44-9ef5-0696f1b03141.png)
Closer look at the splatting result: Points are rendered as oriented disks


## Mesh

```cpp
#include <glk/mesh.hpp>

std::vector<Eigen::Vector3f> vertices = ...;
std::vector<Eigen::Vector3f> normals = ...;
std::vector<Eigen::Vector4f> colors = ...;
std::vector<Eigen::Vector2f> tex_coords;
std::vector<unsigned int> indices;

// Create a mesh instance
// Pass nullptr if normal/color/tex_coord is not available
auto mesh = std::make_shared<glk::Mesh>(
  vertices.data(), sizeof(float) * 3,
  normals.data(), sizeof(float) * 3,
  colors.data(), sizeof(float) * 4,
  tex_coords.data(), sizeof(float) * 2,
  vertices.size()
  indices.data(),
  indices.size()
);

std::shared_ptr<glk::Texture> texture = ...;
mesh->set_texture(texture);
```

## 2D Drawings

**guik::HoveredDrawings** projects 3D object positions on the screen and draws 2D primitives on the projected positions.

```cpp
#include <guik/hovered_drawings.hpp>

// Create hovered drawings renderer and register it to the viewer
auto hovered = std::make_shared<guik::HoveredDrawings>();
viewer->register_ui_callback("hovered", hovered->create_callback());

// Draw a text at a fixed 3D position (1.0, 2.0, 3.0)
std::uint32_t fg_color = IM_COL32(255, 255, 255, 255);
std::uint32_t bg_color = IM_COL32(0, 0, 0, 128);
hovered->add_text({1.0f, 2.0f, 3.0f}, "text1", fg_color, bg_color);

// Instead of directly giving a 3D position, a drawable name can be 
// used to draw a 2D drawing on the drawable position
hovered->add_text_on("drawable_name", "text2", fg_color, bg_color);


// Cross
Eigen::Vector3f position = {1.0f, 2.0f, 3.0f};
std::uint32_t color = IM_COL32(255, 255, 255, 255);
float size = 10.0f;
hovered->add_cross(position, color, size);

// Circle
float radius = 10.0f;
hovered->add_circle(position, color, radius);

// Triangle
float height = 20.0f;
hovered->add_triangle(position, color, height);
hovered->add_filled_triangle(position, color, height);

// Rectangle
Eigen::Vector2f size = {10.0f, 10.0f};
Eigen::Vector2f offset = {0.0f, 0.0f};
hovered->add_rect(position, color, size, offset);
hovered->add_filled_rect(position, color, size, offset);

// Image (glk::Texture)
std::make_shared<glk::Texture> texture = ...;
hovered->add_image(position, texture, size, offset);
```

![Screenshot_20221231_183315](https://user-images.githubusercontent.com/31344317/210131927-75c87acf-a85d-4c8c-a877-1ae8d18e15ad.png)


**guik::HoveredDrawings** can be drawn on subviewers.

```cpp
auto sub = viewer->sub_viewer("sub");
auto hovered = std::make_shared<guik::HoveredDrawings>(sub);
hovered->add_rect_on("coord", IM_COL32(0, 255, 0, 255));
sub->register_ui_callback("hovered", hovered->create_callback());
```


## Image (2D texture)

```cpp
#include <glk/texture.hpp>

// Create a texture from raw pixel data
Eigen::Vector2i size = ...;
GLuint internal_format = GL_RGBA;
GLuint format = GL_RGB;
GLuint type = GL_UNSIGNED_BYTE;
std::vector<unsigned char> pixels = ...;
auto texture = std::make_shared<glk::Texture>(size, internal_format, format, type, pixels.data());

// Register the image to the viewer
viewer->update_image("image", texture);
```

There is also a utility function to create a texture from ```cv::Mat```.
```cpp
#include <glk/texture_opencv.hpp>

cv::Mat image = ...;
auto texture = glk::create_texture(image);

viewer->update_image("image", texture);
```

![Screenshot_20230101_011615](https://user-images.githubusercontent.com/31344317/210149508-e98dd695-9a38-4bd9-8216-96aa5c2510d1.png)

If an image name contains '/', the string before the slash is recognized as a group name, and images with the same group name are grouped in a tab.

```cpp
viewer->update_image("group0/image0", texture);
viewer->update_image("group0/image1", texture);
viewer->update_image("group1/image0", texture);
```

![Screenshot_20230101_011918](https://user-images.githubusercontent.com/31344317/210149509-c096fdcb-7337-44bf-833d-ce369378bbe1.png)


## Plots (ImPlot)

```cpp
#include <implot.h>
#include <guik/viewer/light_viewer.hpp>


std::vector<double> xs = ...;
std::vector<double> ys = ...;

// Basic plotting
viewer->setup_plot("curves_y", 1024, 256);
viewer->update_plot_line("curves_y", "sin", ys_sin);  // When only Y values are given, X values become index IDs
viewer->update_plot_stairs("curves_y", "sin_stairs", ys_sin);


std::vector<double> xs_circle = ...;
std::vector<double> ys_circle = ...;

// If a plot name contains "/", the string before the slash is recognized as a group name.
// Plots with the same group name are displayed in the same tab.
viewer->setup_plot("group02/circle", 1024, 1024, ImPlotFlags_Equal);
viewer->update_plot_line("group02/circle", "circle", xs_circle, ys_circle, ImPlotLineFlags_Loop);
```

![Screenshot_20231212_103830](https://github.com/koide3/iridescence/assets/31344317/0036c0ab-63bb-451f-8f0f-e8931c820f71)
