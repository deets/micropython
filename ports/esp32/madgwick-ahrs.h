#pragma once

typedef struct {
  float beta;				// algorithm gain
  float q0, q1, q2, q3;
  float sample_freq;
} madgwick_data_t; // quaternion of sensor frame relative to auxiliary frame

void madgwick_ahrs_init(madgwick_data_t* state, float sample_freq);
void madgwick_ahrs_update_imu(madgwick_data_t* state, float gx, float gy, float gz, float ax, float ay, float az);
void madgwick_ahrs_compute_angles(madgwick_data_t* state, float* roll, float* pitch, float* yaw);
