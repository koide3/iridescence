#version 330

uniform sampler2D weight_sampler;
uniform sampler2D position_sampler;
uniform sampler2D normal_sampler;
uniform sampler2D color_sampler;

in vec2 texcoord;

out vec3 position;
out vec3 normal;
out vec3 color;

void main() {
  float w = texture(weight_sampler, texcoord).x;
  position = texture(position_sampler, texcoord).xyz / w;
  normal = normalize(texture(normal_sampler, texcoord).xyz / w);
  color = texture(color_sampler, texcoord).xyz / w;
}