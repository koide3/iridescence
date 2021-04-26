#version 430

const int steps = 4;
const int subsample_steps = 1;

uniform vec2 step_size;
uniform sampler2D depth_sampler;

layout (points) in;
layout (points, max_vertices=16) out;

in vec3 vert_in[];
out vec3 vert_out;

void main(void) {
  for(int i=0; i<steps; i++) {
    for(int j=0; j<steps; j++) {
      vec2 texcoord = vert_in[0].xy + step_size * vec2(i, j) * subsample_steps;
      float depth = texture(depth_sampler, texcoord).x;

      if(depth < 1.0) {
        vert_out = vec3(texcoord, depth);

        vec2 screen_pos = texcoord * 2.0 - vec2(1.0);
        gl_Position = vec4(screen_pos, depth, 1.0);
        EmitVertex();
      }
    }
  }
}