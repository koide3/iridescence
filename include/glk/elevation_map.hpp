#ifndef GLK_ELEVATION_MAP_HPP
#define GLK_ELEVATION_MAP_HPP

#include <GL/gl3w.h>
#include <glk/drawable.hpp>
#include <glk/mesh_rendering_options.hpp>

namespace glk {

/**
 * @brief Builder for an elevation map.
 */
struct ElevationMapBuilder {
public:
  /**
   * @brief Constructor
   * @param resolution Resolution of the map (size of each cell)
   * @param size Size of the map in number of cells (width, height)
   * @param origin Top-left corner of the map in world coordinates
   */
  ElevationMapBuilder(double resolution, const Eigen::Vector2i& size, const Eigen::Vector2f& origin);

  /// @brief Destructor
  ~ElevationMapBuilder();

  /// @brief Calculate the cell indices for a given set of cells.
  /// @param cells   Cells for which to calculate the indices. cell[0] and cell[1] are the x and y coordinates of the cell in world coordinates.
  /// @return        Indices of the cells in the elevation map.
  template <typename Vector>
  std::vector<std::uint64_t> calc_cell_indices(const std::vector<Vector>& cells) const;

  /// @brief Set the elevation values for the given cells in the elevation map.
  /// @param cells             Cells
  /// @param elevation_index   Index of the elevation value in the cell vector (i.e., cells[i][elevation_index] is the elevation value of i-th cell).
  /// @param cell_indices      Indices of the cells in the elevation map.
  /// @param average           If false, the elevation values will be set directly. Otherwise, the elevation values of each cell will be averaged.
  template <typename Vector>
  void set_elevation(const std::vector<Vector>& cells, int elevation_index, const std::vector<std::uint64_t>& cell_indices, bool average = false);

  /// @brief Set the color values for the given cells in the elevation map.
  /// @param cells          Cells
  /// @param cmap_index     Index of the color value in the cell vector (i.e., cells[i][cmap_index] is the color value of i-th cell).
  /// @param cell_indices   Indices of the cells in the elevation map.
  /// @param average        If false, the color values will be set directly. Otherwise, the color values of each cell will be averaged.
  template <typename Vector>
  void set_cmap(const std::vector<Vector>& cells, int cmap_index, const std::vector<std::uint64_t>& cell_indices, bool average = false);

  /// @brief Apply local averaging filter to the elevation (and color) map.
  /// @tparam WindowSize Size of the filter window (must be odd).
  /// @param process_valid_cells If true, all cells will be processed (smoothed). If false, only invalid cells (NaN) will be processed.
  template <int WindowSize = 3>
  void filter_elevation_map(bool process_valid_cells = true);

  /// @brief Replace the color map with its gradient magnitude.
  template <int WindowSize = 3>
  void calc_cmap_gradient();

public:
  double resolution;      // Resolution of the map
  double inv_resolution;  // Inverse of the resolution

  Eigen::Vector2i size;    // Size of the map in number of cells (width, height)
  Eigen::Vector2f origin;  // Top-left corner of the map in world coordinates

  Eigen::Array<float, -1, -1, Eigen::RowMajor> count_map;      // Count of points in each cell
  Eigen::Array<float, -1, -1, Eigen::RowMajor> elevation_map;  // Elevation values for each cell
  Eigen::Array<float, -1, -1, Eigen::RowMajor> color_map;      // Color values for each cell (optional)
};

/// @brief Elevation map.
class ElevationMap : public glk::Drawable {
public:
  /// @brief Constructor
  /// @param builder  Builder containing the elevation map data
  /// @param options  Rendering options
  ElevationMap(const ElevationMapBuilder& builder, const MeshRenderingOptions& options = MeshRenderingOptions());

  /// @brief Destructor
  ~ElevationMap();

  /// @brief Draw the elevation map
  void draw(glk::GLSLShader& shader) const;

private:
  MeshRenderingOptions options;  // Rendering options

  double resolution;       // Resolution of the map
  Eigen::Vector2i size;    // Size of the map in number of cells (width, height)
  Eigen::Vector2f origin;  // Top-left corner of the map in world coordinates

  int num_indices_faces;  // Number of indices for faces
  int num_indices_edges;  // Number of indices for edges

  GLuint vao;        // Vertex Array Object
  GLuint vbo;        // Vertex Buffer Object
  GLuint cbo;        // Color Buffer Object
  GLuint ebo_faces;  // Element Buffer Object for faces
  GLuint ebo_edges;  // Element Buffer Object for edges
};

}  // namespace glk

#endif