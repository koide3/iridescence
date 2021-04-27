#version 330
uniform sampler2D depth_sampler;

uniform mat4 inv_projection_view_matrix;

in vec3 vert_out;

layout (location=0) out vec4 frag_position;

vec2 tex2uv(vec2 texcoord) {
  return 2.0 * (texcoord - vec2(0.5, 0.5));
}

vec3 unproject(vec2 uv, float depth) {
  vec4 xyzw = inv_projection_view_matrix * vec4(uv, depth, 1.0);
  return xyzw.xyz / xyzw.w;
}

void main() {
  vec2 texcoord = vert_out.xy;
  float depth = vert_out.z * 2.0 - 1.0;
  frag_position = vec4(unproject(tex2uv(texcoord), depth), depth);
  gl_FragDepth = 1.0;
  // gl_FragDepth = vert_out.z + 1e-3;
}