#ifndef EKF_H
#define EKF_H

typedef struct {
  float q[4];         // q-nion estimate of attitude [q0, q1, q2, q3]
  float g_bias[3];    // gyro bias [bgx, bgy, bgz] [rad/s]
  float P[7][7];      // state covariance matrix
} ekf_state_t;        

void mat_zero(int rows, int cols, float* mat);
void mat_eye(int n, float* mat);
void mat_copy(int rows, int cols, const float* src, float* dst);

void mat_add(int rows, int cols, const float* A, const float* B, float* result);
void mat_sub(int rows, int cols, const float* A, const float* B, float* result);
void mat_mult(int m, int n, int p, const float* A, const float* B, float* result);
void mat_transpose(int rows, int cols, const float* A, float* result);
void mat_scale(int rows, int cols, const float* A, float scalar, float* result);

int mat_inv33(const float* A, float* result);

void quat_normalize(float q[4]);
void quat_multiply(const float q1[4], const float q2[4], float result[4]);
void quat_to_euler(const foat q[4], float* roll, float* pitch, float* yaw);

void ekf_init(ekf_state_t* ekf);
void ekf_predict(ekf_state_t* ekf, float gx, float gy, float gz);
void ekf_get_attitude(const ekf_state_t* ekf, float* roll, float* pitch, float* yaw);

#endif
