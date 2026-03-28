#version 430

layout (points) in;
layout (triangle_strip, max_vertices=24) out;

uniform float resolution;
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

in vec3 frag_vert_position[];
in vec3 frag_face_layers[];   // (top_layer, side_layer, bottom_layer)

out vec3 frag_normal;
out vec3 frag_texcoord3d;  // (u, v, texture_array_layer)

void emit_face(in mat4 mvp, in mat3 normal_matrix, in vec3 face_normal,
               in vec3 a, in vec3 b, in vec3 c, in vec3 d, in float tex_layer, in bool flip_v) {
  frag_normal = normal_matrix * face_normal;

  float v0 = flip_v ? 1.0 : 0.0;
  float v1 = flip_v ? 0.0 : 1.0;

  frag_texcoord3d = vec3(0.0, v0, tex_layer);
  gl_Position = mvp * vec4(a, 1.0);
  EmitVertex();

  frag_texcoord3d = vec3(1.0, v0, tex_layer);
  gl_Position = mvp * vec4(b, 1.0);
  EmitVertex();

  frag_texcoord3d = vec3(0.0, v1, tex_layer);
  gl_Position = mvp * vec4(c, 1.0);
  EmitVertex();

  frag_texcoord3d = vec3(1.0, v1, tex_layer);
  gl_Position = mvp * vec4(d, 1.0);
  EmitVertex();

  EndPrimitive();
}

void main() {
  vec3 o = frag_vert_position[0];
  float r = resolution;

  float top_layer    = frag_face_layers[0].x;
  float side_layer   = frag_face_layers[0].y;
  float bottom_layer = frag_face_layers[0].z;

  // 8 corners of the voxel cube
  vec3 v0 = o;                    // (0, 0, 0)
  vec3 v1 = o + vec3(r, 0, 0);   // (r, 0, 0)
  vec3 v2 = o + vec3(r, r, 0);   // (r, r, 0)
  vec3 v3 = o + vec3(0, r, 0);   // (0, r, 0)
  vec3 v4 = o + vec3(0, 0, r);   // (0, 0, r)
  vec3 v5 = o + vec3(r, 0, r);   // (r, 0, r)
  vec3 v6 = o + vec3(r, r, r);   // (r, r, r)
  vec3 v7 = o + vec3(0, r, r);   // (0, r, r)

  mat4 mvp = projection_matrix * view_matrix * model_matrix;
  mat3 normal_matrix = transpose(inverse(mat3(model_matrix)));

  if (top_layer < 0.0) {
    // Cross-block rendering: two intersecting quads (flowers, grasses, saplings, etc.)
    float tex_layer = side_layer;
    const float inv_sqrt2 = 0.70710678;

    // Quad 1: diagonal from (0,0) to (r,r) in XY, spanning full Z height
    vec3 n1 = vec3(inv_sqrt2, -inv_sqrt2, 0.0);
    emit_face(mvp, normal_matrix,  n1, v0, v2, v4, v6, tex_layer, true);  // Front
    emit_face(mvp, normal_matrix, -n1, v2, v0, v6, v4, tex_layer, true);  // Back

    // Quad 2: diagonal from (r,0) to (0,r) in XY, spanning full Z height
    vec3 n2 = vec3(inv_sqrt2, inv_sqrt2, 0.0);
    emit_face(mvp, normal_matrix,  n2, v1, v3, v5, v7, tex_layer, true);  // Front
    emit_face(mvp, normal_matrix, -n2, v3, v1, v7, v5, tex_layer, true);  // Back
  } else {
    // Solid cube rendering: 6 faces [Z-up coordinate system]
    emit_face(mvp, normal_matrix, vec3( 0,  0,  1), v4, v5, v7, v6, top_layer,    false);  // Top    (+Z)
    emit_face(mvp, normal_matrix, vec3( 0,  0, -1), v0, v3, v1, v2, bottom_layer, false);  // Bottom (-Z)
    emit_face(mvp, normal_matrix, vec3( 1,  0,  0), v1, v2, v5, v6, side_layer,   true);   // Side   (+X)
    emit_face(mvp, normal_matrix, vec3(-1,  0,  0), v3, v0, v7, v4, side_layer,   true);   // Side   (-X)
    emit_face(mvp, normal_matrix, vec3( 0,  1,  0), v2, v3, v6, v7, side_layer,   true);   // Side   (+Y)
    emit_face(mvp, normal_matrix, vec3( 0, -1,  0), v0, v1, v4, v5, side_layer,   true);   // Side   (-Y)
  }
}
