#version 330

layout (points) in;
layout (triangle_strip, max_vertices=4) out;

uniform sampler2D position_sampler;
uniform sampler2D radius_sampler;
uniform sampler2D normal_sampler;
uniform sampler2D minor_tangent_sampler;
uniform sampler2D major_tangent_sampler;
uniform sampler2D color_sampler;

uniform int pass_stage;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform mat4 projection_view_matrix;

in vec3 vert_out[];

out vec2 texcoord;
out vec3 frag_position;
out vec3 frag_normal;
out vec3 frag_color;

void main() {
  vec3 uvd = vert_out[0];

  vec3 radii = texture(radius_sampler, uvd.xy).xyz;
  if(radii.y < 1e-3 || radii.z > 0.5) {
    return;
  }

  float minor_r = radii[1] * 2.0;
  float major_r = radii[2] * 2.0;

  vec3 center = texture(position_sampler, uvd.xy).xyz;
  vec3 normal = normalize(texture(normal_sampler, uvd.xy).xyz);
  vec3 minor = normalize(texture(minor_tangent_sampler, uvd.xy).xyz) * minor_r;
  vec3 major = normalize(texture(major_tangent_sampler, uvd.xy).xyz) * major_r;

  float depth_offset = 0.0;
  if(pass_stage == 0) {
    float center_z = (view_matrix * vec4(center, 1.0)).z;
    float center_z_shifted = center_z - major_r;
    vec4 center_z_ncd = projection_matrix * vec4(0.0, 0.0, center_z, 1.0);
    vec4 center_z_ncd_shifted = projection_matrix * vec4(0.0, 0.0, center_z_shifted, 1.0);

    float center_depth = center_z_ncd.z / center_z_ncd.w;
    float center_depth_shifted = center_z_ncd_shifted.z / center_z_ncd_shifted.w;
    depth_offset = center_depth_shifted - center_depth;
  }

  frag_normal = normalize(normal);
  frag_color = texture(color_sampler, uvd.xy).xyz;
  frag_position = center - minor + major;
  gl_Position = projection_view_matrix * vec4(frag_position, 1.0);
  gl_Position += vec4(0.0, 0.0, gl_Position.w * depth_offset, 0.0);
  texcoord = vec2(-1.0, 1.0);
  EmitVertex();

  frag_position = center - minor - major;
  gl_Position = projection_view_matrix * vec4(frag_position, 1.0);
  gl_Position += vec4(0.0, 0.0, gl_Position.w * depth_offset, 0.0);
  texcoord = vec2(-1.0, -1.0);
  EmitVertex();

  frag_position = center + minor + major;
  gl_Position = projection_view_matrix * vec4(frag_position, 1.0);
  gl_Position += vec4(0.0, 0.0, gl_Position.w * depth_offset, 0.0);
  texcoord = vec2(1.0, 1.0);
  EmitVertex();

  frag_position = center + minor - major;
  gl_Position = projection_view_matrix * vec4(frag_position, 1.0);
  gl_Position += vec4(0.0, 0.0, gl_Position.w * depth_offset, 0.0);
  texcoord = vec2(1.0, -1.0);
  EmitVertex();
}