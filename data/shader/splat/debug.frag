#version 330

uniform sampler2D sampler0;
uniform sampler2D sampler1;
uniform sampler2D sampler2;
uniform sampler2D sampler3;

in vec2 texcoord;
out vec4 color;

void compute_eigen(vec3 c1, vec3 c2, out vec3 eval, out mat3 evec);
void sort_eigen(inout vec3 eval, inout mat3 evec);


void main() {
  float num_points = texture(sampler0, texcoord).x;

  if(num_points < 1) {
    discard;
  }

  vec4 values1 = texture(sampler1, texcoord);
  vec3 cross1 = texture(sampler2, texcoord).xyz;
  vec3 cross2 = texture(sampler3, texcoord).xyz;

  vec3 eval;
  mat3 evec;
  compute_eigen(cross1, cross2, eval, evec);
  sort_eigen(eval, evec);

  if(eval[0] > eval[1] || eval[0] > eval[2] || eval[1] > eval[2]) {
    color = vec4(1.0, 0.0, 0.0, 1.0);
  } else {
    color = vec4(1.0);
  }

  // color = vec4(evec[0], 1.0) * 100.0;
  return;


  mat3 cov;
  cov[0] = vec3(cross1[0], cross1[1], cross1[2]);
  cov[1] = vec3(cross1[1], cross2[0], cross2[1]);
  cov[2] = vec3(cross1[2], cross2[1], cross2[2]);

  mat3 inv_cov = inverse(cov + mat3(1e-6));
  vec3 normal = vec3(1.0);
  for(int i = 0; i < 3; i++) {
    normal = normalize(inv_cov * normal);
  }

  color = vec4(abs(normal), 1.0);

  // vec3 trace = vec3(cov[0][0], cov[1][1], cov[2][2]);
  // color = vec4(trace / length(trace), 1.0);
  // color = vec4(cov[0][0], cov[1][1], cov[2][2], 1.0) * 10.0;
  return;
}