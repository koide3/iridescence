#version 330

in vec2 texcoord;
in vec3 frag_position;
in vec3 frag_normal;
in vec3 frag_color;

out float weight;
out vec3 position;
out vec3 normal;
out vec3 color;

void main() {
  float r = length(texcoord);
  if(r > 1.0) {
    discard;
  }

  // float w = 1.0 - r;
  float w = exp(-5.0 * r);

  weight = w;
  position = w * frag_position;
  normal = w * frag_normal;
  color = w * frag_color;
}