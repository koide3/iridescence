#version 330
uniform bool normal_enabled;

// point_scale_mode = 0 : screen space scale
// point_scale_mode = 1 : metric scale
// point_shape_mode = 0 : rectangle
// point_shape_mode = 1 : circle
uniform int point_scale_mode;
uniform int point_shape_mode;
uniform float point_size;
uniform float point_scale;
uniform float point_size_offset;

uniform vec2 viewport_size;
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

// colormode = 0 : rainbow (height encoding)
// colormode = 1 : material_color
// colormode = 2 : vert_color
// colormode = 3 : texture_color
// colormode = 4 : vert_cmap (color map)
uniform int color_mode;
uniform vec4 material_color;
uniform sampler2D colormap_sampler;
uniform sampler2D texture_sampler;

uniform vec2 z_range;
uniform vec2 cmap_range;
uniform vec3 colormap_axis;

in vec3 vert_position;
in vec4 vert_color;
in vec2 vert_texcoord;
in vec3 vert_normal;
in float vert_cmap;

out vec4 frag_color;
out vec2 frag_texcoord;
out vec3 frag_normal;

vec4 rainbow(float val, vec2 range) {
    float p = (val - range[0]) / (range[1] - range[0]);
    return texture(colormap_sampler, vec2(p, 0.0));
}

void main() {
    vec4 world_position = model_matrix * vec4(vert_position, 1.0);
    vec3 frag_world_position = world_position.xyz;
    gl_Position = projection_matrix * view_matrix * world_position;

    switch(color_mode) {
        case 0:
            frag_color = rainbow(dot(frag_world_position, colormap_axis), z_range);
            frag_color.a = material_color.a;
            break;

        case 1:
            frag_color = material_color;
            break;

        case 2:
            frag_color = vert_color;
            break;

        case 3:
            frag_texcoord = vert_texcoord;
            break;

        case 4:
            frag_color = rainbow(vert_cmap, cmap_range);
            frag_color.a = material_color.a;
            break;
    }

    if(normal_enabled) {
        mat3 normal_matrix = transpose(inverse(mat3(model_matrix)));
        frag_normal = normal_matrix * vert_normal;
    } else {
        frag_normal = vec3(0.0, 0.0, 0.0);
    }

    if (point_scale_mode == 0) {
        vec3 ndc = gl_Position.xyz / gl_Position.w;
        float z_dist = 1.0 - ndc.z;
        gl_PointSize = point_scale * point_size * z_dist + point_size_offset;
    } else {
        float point_radius = point_scale * point_size + point_size_offset;
        gl_PointSize = viewport_size[1] * projection_matrix[1][1] * point_radius / gl_Position.w;
    }
}