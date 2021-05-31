#version 330

uniform sampler2D sampler0;
uniform sampler2D sampler1;
uniform sampler2D sampler2;

in vec2 texcoord;
out vec4 color;

void main() {
  color = texture(sampler1, texcoord);

  // color = vec4(weight / 5.0, 0.0, 0.0, 1.0);
  // color = vec4(normal, 1.0);

  // color = vec4(weight / 2.0, 0.0, 0.0, 1.0);
}