#version 430

in vec3 vert_position;
out vec3 vert_in;

void main() {
  vert_in = vert_position;
}