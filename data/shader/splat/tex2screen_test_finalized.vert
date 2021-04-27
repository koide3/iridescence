#version 330

uniform sampler2D finalized_radius_sampler;

in vec3 vert_position;

out vec3 uvd;
out vec2 screen_pos;

void main() {
    uvd = vert_position;
    screen_pos = vert_position.xy * 2.0 - vec2(1.0);

    float finalized_radius = texture(finalized_radius_sampler, uvd.xy).x;
    if(finalized_radius > 1e-3) {
      gl_Position = vec4(0.0, 0.0, 10.0, 1.0);
      gl_PointSize = 1.0;
      return;
    }

    gl_Position = vec4(screen_pos, vert_position.z, 1.0);
    gl_PointSize = 1.0;
}