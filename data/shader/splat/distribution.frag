#version 330

uniform sampler2D position_sampler;
uniform sampler2D radius_sampler;

uniform vec2 inv_screen_size;

in vec3 p1_pos;
in float p1_radius;
in float p1_radius_screen;

out float feedback_radius;

void main() {
  vec2 p2_texcoord = gl_FragCoord.xy * inv_screen_size;
  vec3 p2_pos = texture(position_sampler, p2_texcoord).xyz;

  if(length(p1_pos - p2_pos) > p1_radius) {
    discard;
  }

  feedback_radius = p1_radius;
}