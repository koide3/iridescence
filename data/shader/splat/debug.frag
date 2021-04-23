#version 330

uniform sampler2D sampler0;
uniform sampler2D sampler1;
uniform sampler2D sampler2;

in vec2 texcoord;
out vec4 color;

void main() {
  vec4 values0 = texture(sampler0, texcoord);
  vec4 values1 = texture(sampler1, texcoord);
  vec4 values2 = texture(sampler2, texcoord);

  if(values0.y < 1e-3) {
    discard;
  }

  if(abs(values2.w - 10) > 2.5) {
    color = vec4(1.0, 0.0, 0.0, 1.0);
  } else {
    color = vec4(0.0, 1.0, 0.0, 1.0);
  }

  /*
  if(abs(values2.w - 10) <= 2.5) {
    color = vec4(1.0);
  } else {
    color = vec4(1.0, 0.0, 0.0, 1.0);
  }
  */

  // color = abs(values0 - values1);

  // color = values0 - values1;



  // color = values1;

  // color = values1;
}