#include <math.h>
#include <string.h>
#include "ekf.h"

/*
* state vector [7x1]: 
*   quat     : q0, q1, q2, q3 -> orientation component
*   gyro bias: bgx, bgy, bgz  -> gyro bias
*     - main contributor to drift
*     - measured_gyro = true_w + bias + noise
*     - caused by temp changes, age, power supply variation, mech. stress
*     - factory calibration (once during startup) is not sufficient!
*
*   use EKF instead to estimate bias continously as a state variable
*     - w_corrected = w_measured - ekf.bg 
*     - adapts to changing bias during operation
*
* 
*   predict step: bg_new = bg_old + process_noise
*   update step : 
*     - accelerometer provides gravity direction
*     - if attitude prediction is wrong, gyro must have bias
*     - EKF adjusts bias estiamte to make prediction match measurement
*
*     - so if r/p are drifting, an accel reading can be used to correct them
*/


// 2D arrays are stored as 1D arrays in memory where idx = (i * cols) + j   
// float* mat is a pointer to the first element of the allocated 1D array
// to specify the offsets, we require the dimensions of the 2D array
void mat_zero(int rows, int cols, float* mat) {

  for(int i = 0; i < rows * cols; i++)  {
    mat[i] = 0.0f;
  }
}

void mat_eye(int n, float* mat) {
  mat_zero(n, n, mat);
  for(int i = 0; i < n; i++) {
    mat[(n*i) + i] = 1.0f;
  }
}

void mat_copy(int rows, int cols, const float* src, float* dst) {
  memcpy(dst, src, rows * cols * sizeof(float));
}

void mat_add(int rows, int cols, const float* A, const float* B, float* result) {
  for(int i = 0; i < rows * cols; i++) {
    result[i] = A[i] + B[i];
  }
}

void mat_sub(int rows, int cols, const float* A, const float* B, float* result) {
  for(int i = 0; i < rows * cols; i++) {
    result[i] = A[i] - B[i];
  }
}

void mat_scale(int rows, int cols, const float* A, float scalar, float* result) {
  for(int i = 0; i < rows * cols; i++) {
    result[i] = scalar * A[i];
  }
}

void mat_mult(int m, int n, int p, const float* A, const float* B, float* result) {
  for(int i = 0; i < m; i++) {
    for(int j = 0; j < p; j++) {
      float sum = 0.0f;
      for(int k = 0; k < n; k++) {
        sum += A[i * n + k] * B[k * p + j];
      }

      result[i * p + j] = sum;
    }
  }
}

void mat_transpose(int rows, int cols, const float* A, float* result) {
  for(int i = 0; i < rows; i++) {
    for(int j = 0; j < cols; j++) {
      result[j * rows + i] = A[i * cols + j];
    }
  }
}

int mat_inv33(const float* A, float* result) {
  // Compute determinant
  float det = A[0] * (A[4]*A[8] - A[5]*A[7])
            - A[1] * (A[3]*A[8] - A[5]*A[6])
            + A[2] * (A[3]*A[7] - A[4]*A[6]);

  // Check for singularity
  if(fabsf(det) < 1e-10f) {
    return 0;  // Matrix is singular (non-invertible)
  }

  float inv_det = 1.0f / det;

  // Compute adjugate matrix and scale by 1/det
  result[0] = (A[4]*A[8] - A[5]*A[7]) * inv_det;
  result[1] = (A[2]*A[7] - A[1]*A[8]) * inv_det;
  result[2] = (A[1]*A[5] - A[2]*A[4]) * inv_det;

  result[3] = (A[5]*A[6] - A[3]*A[8]) * inv_det;
  result[4] = (A[0]*A[8] - A[2]*A[6]) * inv_det;
  result[5] = (A[2]*A[3] - A[0]*A[5]) * inv_det;

  result[6] = (A[3]*A[7] - A[4]*A[6]) * inv_det;
  result[7] = (A[1]*A[6] - A[0]*A[7]) * inv_det;
  result[8] = (A[0]*A[4] - A[1]*A[3]) * inv_det;

  return 1;  // Success
}

void quat_normalize(float q[4]) {
  float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

  if(fabsf(norm) < 1e-10f) {
    // Degenerate quaternion, reset to identity
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }

  float inv_norm = 1.0f / norm;
  q[0] *= inv_norm;
  q[1] *= inv_norm;
  q[2] *= inv_norm;
  q[3] *= inv_norm;
}

void quat_multiply(const float q1[5], const float q2[4], float result[4]) {
// quaternion multiplcation composes two rotations (called hamilton product)
// right-to-left rotation order: imlpement q1 then q2 by doing q2 * q1 *q0 (eye)
 
// q = [q0,q1,q2,q3] = [w,x,y,z] where:
  //  - w: scalar component, 
  //  - [x,y,z]: vector component
  
// -------- hamilton product rules ------------- 
  // q = w + xi + yj + zk where i,j,k are imaginary numbers such that:
    //   i² = j² = k² = -1
    //   i*j = k,   j*i = -k
    //   j*k = i,   k*j = -i
    //   k*i = j,   i*k = -j
// -------- hamilton product rules ------------- 
  

}
