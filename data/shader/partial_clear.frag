#version 330
uniform usampler2D dynamic_flag_sampler;

uniform vec4 clear_color;
uniform bool info_enabled;
uniform bool normal_enabled;

in vec2 texcoord;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;
layout (location=2) out vec3 normal;

void main() {
  uint dynamic_flag = texture(dynamic_flag_sampler, texcoord).r;
  if(dynamic_flag == uint(0)) {
    discard;
  }

  color = clear_color;
  
  if(info_enabled) {
    info = ivec4(-1, -1, -1, -1);
  }

  if(normal_enabled) {
    normal = vec3(0.0, 0.0, 0.0);
  }

  gl_FragDepth = 1.0;
}