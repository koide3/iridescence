#version 330

uniform vec4 material_color;

in vec3 vert_position;
in vec3 vert_normal;
in vec4 vert_color;
in float vert_radius;

out VertexData {
  vec3 position;
  vec3 normal;
  vec4 color;
  float radius;
} vertex_out;

void main() {
  vertex_out.position = vert_position;
  vertex_out.normal = vert_normal;
  vertex_out.color = vert_color;
  vertex_out.radius = vert_radius;
}