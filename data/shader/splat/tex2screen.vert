#version 330

in vec3 vert_position;

out vec3 uvd;
out vec2 screen_pos;

void main() {
    uvd = vert_position;
    screen_pos = vert_position.xy * 2.0 - vec2(1.0);
    gl_Position = vec4(screen_pos, vert_position.z, 1.0);
    gl_PointSize = 1.0;
}