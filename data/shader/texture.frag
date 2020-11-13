#version 330
uniform sampler2D color_sampler;

in vec2 texcoord;
out vec4 color;

void main() {
  color = texture(color_sampler, texcoord);
}