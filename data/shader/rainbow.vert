#version 330
uniform float point_size;
uniform float point_scale;
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

// colormode = 0 : rainbow (height encoding)
// colormode = 1 : material_color
// colormode = 2 : vert_color
uniform int color_mode;
uniform vec4 material_color;
uniform sampler1D colormap_sampler;

uniform vec2 z_range;
uniform vec3 colormap_axis;

in vec3 vert_position;
in vec4 vert_color;

out vec4 frag_color;
out vec3 frag_world_position;

vec4 rainbow(vec3 position) {
    float p = (dot(position, colormap_axis) - z_range[0]) / (z_range[1] - z_range[0]);
    return texture(colormap_sampler, p);
}

void main() {
    vec4 world_position = model_matrix * vec4(vert_position, 1.0);
    frag_world_position = world_position.xyz;
    gl_Position = projection_matrix * view_matrix * world_position;

    if(color_mode == 0) {
        frag_color = rainbow(frag_world_position);
    } else if(color_mode == 1) {
        frag_color = material_color;
    } else if(color_mode == 2) {
        frag_color = vert_color;
    }

    vec3 ndc = gl_Position.xyz / gl_Position.w;
    float z_dist = 1.0 - ndc.z;
    gl_PointSize = point_scale * point_size * z_dist;
}