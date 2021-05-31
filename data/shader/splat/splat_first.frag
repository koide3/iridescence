#version 330

in vec2 texcoord;

out float weight;
out vec3 position;
out vec3 normal;

void main() {
  float r = length(texcoord);
  if(r > 1.0) {
    discard;
  }
}