/**
 * This GLSL code is written based on the C++ code accompanied to [1]
 * The original license disclaimer follows (Boost Software License)
 * [1] David, Eberly, A Robust Eigensolver for3×3Symmetric Matrices
 *
 * David Eberly, Geometric Tools, Redmond WA 98052
 * Copyright (c) 1998-2021
 * Distributed under the Boost Software License, Version 1.0.
 * https://www.boost.org/LICENSE_1_0.txt
 * https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
 * Version: 4.0.2019.08.16
 */

#version 330

// forward declaration
// c1 = a00, a01, a02
// c2 = a11, a12, a22
void compute_eigen(vec3 c1, vec3 c2, out vec3 eval, out mat3 evec);

// increasing order
void sort_eigen(inout vec3 eval, inout mat3 evec);

void compute_orthogonal_complement(vec3 W, inout vec3 U, inout vec3 V);
void compute_eigen_vector0(float a00, float a01, float a02, float a11, float a12, float a22, float eval0, out vec3 evec0);
void compute_eigen_vector1(float a00, float a01, float a02, float a11, float a12, float a22, vec3 evec0, float eval1, out vec3 evec1);


void compute_eigen(vec3 c1, vec3 c2, out vec3 eval, out mat3 evec) {
  vec3 max_abs = max(abs(c1), abs(c2));
  float max_abs_element = max(max(max_abs.x, max_abs.y), max_abs.z);

  if(max_abs_element < 1e-6) {
    // A is zero
    eval = vec3(0.0);
    evec = mat3(1.0);
    return;
  }

  float inv_max_abs_element = 1.0 / max_abs_element;

  c1 = c1 * inv_max_abs_element;
  c2 = c2 * inv_max_abs_element;

  float a00 = c1[0];
  float a01 = c1[1];
  float a02 = c1[2];
  float a11 = c2[0];
  float a12 = c2[1];
  float a22 = c2[2];

  float norm = dot(vec3(a01, a02, a12), vec3(a01, a02, a12));

  if(norm < 1e-6) {
    // A is diagonal
    eval = vec3(a00, a11, a22) * max_abs_element;
    evec = mat3(1.0);
    return;
  }

  // compute the eigenvalues
  float q = (a00 + a11 + a22) / 3.0;
  float b00 = a00 - q;
  float b11 = a11 - q;
  float b22 = a22 - q;

  float p = sqrt((b00 * b00 + b11 * b11 + b22 * b22 + norm * 2.0) / 6.0);

  float c00 = b11 * b22 - a12 * a12;
  float c01 = a01 * b22 - a12 * a02;
  float c02 = a01 * a12 - b11 * a02;
  float det = (b00 * c00 - a01 * c01 + a02 * c02) / (p * p * p);

  float half_det = clamp(det * 0.5, -1.0, 1.0);

  const float two_thirds_pi = 2.09439510239319549;
  float angle = acos(half_det) / 3.0;
  float beta2 = cos(angle) * 2.0;
  float beta0 = cos(angle + two_thirds_pi) * 2.0;
  float beta1 = -(beta0 + beta2);

  eval = vec3(q) + vec3(p) * vec3(beta0, beta1, beta2);

  vec3 evec0, evec1, evec2;
  if(half_det >= 0.0) {
    compute_eigen_vector0(a00, a01, a02, a11, a12, a22, eval[2], evec2);
    compute_eigen_vector1(a00, a01, a02, a11, a12, a22, evec2, eval[1], evec1);
    evec0 = cross(evec1, evec2);
  } else {
    compute_eigen_vector0(a00, a01, a02, a11, a12, a22, eval[0], evec0);
    compute_eigen_vector1(a00, a01, a02, a11, a12, a22, evec0, eval[1], evec1);
    evec2 = cross(evec0, evec1);
  }

  evec[0] = evec0;
  evec[1] = evec1;
  evec[2] = evec2;

  eval = eval * max_abs_element;
}

void compute_orthogonal_complement(vec3 W, inout vec3 U, inout vec3 V) {
  if(abs(W[0]) > abs(W[1])) {
    float inv_length = 1.0 / sqrt(W[0] * W[0] + W[2] * W[2]);
    U = vec3(-W[2] * inv_length, 0.0, W[0] * inv_length);
  } else {
    float inv_length = 1.0 / sqrt(W[1] * W[1] + W[2] * W[2]);
    U = vec3(0.0, W[2] * inv_length, -W[1] * inv_length);
  }
  V = cross(W, U);
}

void compute_eigen_vector0(float a00, float a01, float a02, float a11, float a12, float a22, float eval0, out vec3 evec0) {
  vec3 row0 = vec3(a00 - eval0, a01, a02);
  vec3 row1 = vec3(a01, a11 - eval0, a12);
  vec3 row2 = vec3(a02, a12, a22 - eval0);

  vec3 r0xr1 = cross(row0, row1);
  vec3 r0xr2 = cross(row0, row2);
  vec3 r1xr2 = cross(row1, row2);

  float d0 = dot(r0xr1, r0xr1);
  float d1 = dot(r0xr2, r0xr2);
  float d2 = dot(r1xr2, r1xr2);

  float dmax = d0;
  int imax = 0;

  if(d1 > dmax) {
    dmax = d1;
    imax = 1;
  }

  if(d2 > dmax) {
    imax = 2;
  }

  switch(imax) {
    case 0:
      evec0 = r0xr1 / sqrt(d0);
      break;
    case 1:
      evec0 = r0xr2 / sqrt(d1);
      break;
    case 2:
      evec0 = r1xr2 / sqrt(d2);
      break;
  }
}

void compute_eigen_vector1(float a00, float a01, float a02, float a11, float a12, float a22, vec3 evec0, float eval1, out vec3 evec1) {
  vec3 U, V;
  compute_orthogonal_complement(evec0, U, V);

  mat3 A;
  A[0] = vec3(a00, a01, a02);
  A[1] = vec3(a01, a11, a12);
  A[2] = vec3(a02, a12, a22);

  vec3 AU = A * U;
  vec3 AV = A * V;

  float m00 = U[0] * AU[0] + U[1] * AU[1] + U[2] * AU[2] - eval1;
  float m01 = U[0] * AV[0] + U[1] * AV[1] + U[2] * AV[2];
  float m11 = V[0] * AV[0] + V[1] * AV[1] + V[2] * AV[2] - eval1;

  float abs_m00 = abs(m00);
  float abs_m01 = abs(m01);
  float abs_m11 = abs(m11);

  if (abs_m00 >= abs_m11) {
    float max_abs_comp = max(abs_m00, abs_m01);
    if (max_abs_comp > 0.0) {
      if (abs_m00 >= abs_m01) {
        m01 /= m00;
        m00 = 1.0 / (1.0 + m01 * m01);
        m01 *= m00;
      } else {
        m00 /= m01;
        m01 = 1.0 / sqrt(1.0 + m00 * m00);
        m00 *= m01;
      }
      evec1 = m01 * U - m00 * V;
    } else {
      evec1 = U;
    }
  } else {
    float max_abs_comp = max(abs_m11, abs_m01);
    if (max_abs_comp > 0.0) {
      if (abs_m11 >= abs_m01) {
        m01 /= m11;
        m11 = 1.0 / sqrt(1.0 + m01 * m01);
        m01 *= m11;
      } else {
        m11 /= m01;
        m01 = 1.0 / sqrt(1.0 + m11 * m11);
        m11 *= m01;
      }
      evec1 = m11 * U - m01 * V;
    } else {
      evec1 = U;
    }
  }
}

void sort_eigen(inout vec3 eval, inout mat3 evec) {
  ivec3 index;

  if(eval[0] < eval[1]) {
    if(eval[2] < eval[0]) {
      index = ivec3(2, 0, 1);
    } else if (eval[2] < eval[1]) {
      index = ivec3(0, 2, 1);
    } else {
      index = ivec3(0, 1, 2);
    }
  } else {
    if(eval[2] < eval[1]) {
      index = ivec3(2, 1, 0);
    } else if (eval[2] < eval[0]) {
      index = ivec3(1, 2, 0);
    } else {
      index = ivec3(1, 0, 2);
    }
  }

  vec3 eval_ = eval;
  mat3 evec_ = evec;

  for(int i=0; i<3; i++) {
    eval[i] = eval_[index[i]];
    evec[i] = evec_[index[i]];
  }
}