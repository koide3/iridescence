#version 330
uniform sampler2D depth_sampler;
uniform sampler2D position_sampler;

uniform vec3 view_point;
uniform vec2 inv_frame_size;

in vec2 texcoord;

layout (location=0) out vec3 frag_normal;

void main() {
  if(texture(depth_sampler, texcoord).x > 1.0 - 1e-5) {
    return;
  }

  vec4 position = texture(position_sampler, texcoord);

  mat3 cov = mat3(0.0);
  for(int i=-1; i<=1; i++) {
    for(int j=-1; j<=1; j++) {
      vec2 coord = texcoord + vec2(i, j) * inv_frame_size;
      vec3 p = texture(position_sampler, coord).xyz;
      vec3 d = position.xyz - p;
      cov += outerProduct(d, d);
    }
  }

  mat3 inv_cov = inverse(cov + mat3(1e-6));

  vec3 pt2view = view_point - position.xyz;
  vec3 normal = normalize(pt2view);
  for(int i=0; i<3; i++) {
    normal = normalize(inv_cov * normal);
  }

  if(dot(normal, pt2view) < 0.0) {
    normal = -normal;
  }

  frag_normal = normal;
}
