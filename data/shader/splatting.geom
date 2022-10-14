#version 330

layout (points) in;
layout (triangle_strip, max_vertices=4) out;

uniform bool vert_radius_enabled;
uniform float point_radius;
uniform int color_mode;
uniform vec2 z_range;
uniform vec3 colormap_axis;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

uniform vec4 material_color;
uniform sampler2D colormap_sampler;

in VertexData {
  vec3 position;
  vec3 normal;
  vec4 color;
  float radius;
} vertex_in[];

out FragmentData {
  vec2 texcoord;
  vec3 position;
  vec3 normal;
  vec4 color;
} vertex_out;


vec4 rainbow(vec3 position) {
    float p = (dot(position, colormap_axis) - z_range[0]) / (z_range[1] - z_range[0]);
    return texture(colormap_sampler, vec2(p, 0.0));
}

void main() {
  vec3 center = vertex_in[0].position;
  vec3 normal = vertex_in[0].normal;

  float radius = point_radius;
  if (vert_radius_enabled) {
    radius = radius * vertex_in[0].radius;
  }

  vec3 major = radius * normalize(abs(normal.y) > abs(normal.x) ? cross(normal, vec3(1.0, 0.0, 0.0)) : cross(normal, vec3(0.0, 1.0, 0.0)));
  vec3 minor = radius * normalize(cross(normal, major));

  mat3 normal_matrix = transpose(inverse(mat3(model_matrix)));
  vertex_out.normal = normal_matrix * normal;

  switch (color_mode) {
    case 0:
      vec3 center_world = (model_matrix * vec4(center, 1.0)).xyz;
      vertex_out.color = rainbow(center_world);
      break;
    case 1:
      vertex_out.color = material_color;
      break;
    case 2:
      vertex_out.color = vertex_in[0].color;
      break;
    case 4:
      vertex_out.color = vec4(1.0);
      break;
  }

  mat4 projection_view_matrix = projection_matrix * view_matrix;
  
  vertex_out.texcoord = vec2(0.0, 1.0);
  vertex_out.position = (model_matrix * vec4(center - minor + major, 1.0)).xyz;
  gl_Position = projection_view_matrix * vec4(vertex_out.position, 1.0);
  EmitVertex();

  vertex_out.texcoord = vec2(0.0, 0.0);
  vertex_out.position = (model_matrix * vec4(center - minor - major, 1.0)).xyz;
  gl_Position = projection_view_matrix * vec4(vertex_out.position, 1.0);
  EmitVertex();

  vertex_out.texcoord = vec2(1.0, 1.0);
  vertex_out.position = (model_matrix * vec4(center + minor + major, 1.0)).xyz;
  gl_Position = projection_view_matrix * vec4(vertex_out.position, 1.0);
  EmitVertex();

  vertex_out.texcoord = vec2(1.0, 0.0);
  vertex_out.position = (model_matrix * vec4(center + minor - major, 1.0)).xyz;
  gl_Position = projection_view_matrix * vec4(vertex_out.position, 1.0);
  EmitVertex();
}
