#ifndef __AHRS_H
#define __AHRS_H

#include "vector_space.h"

/* IMU management structure */
typedef struct {
	vector3d_16_t accel_unscaled_data;
	vector3d_16_t gyro_unscaled_data;

	vector3d_f_t accel_raw_data;
	vector3d_f_t gyro_raw_data;

	vector3d_f_t accel_filtered_data;
	vector3d_f_t gyro_filtered_data;
} imu_data_t;

typedef struct {
	float roll_angle;
	float pitch_angle;
	float yaw_angle;
} attitude_t;

/* AHRS data management structure */
typedef struct {
	attitude_t accel_attitude;
	attitude_t gyro_attitude;

	attitude_t fusion_attitude;
} ahrs_data_t;

void gyro_integrate(attitude_t *attitude, vector3d_f_t gyro_scaled_data,
	float period_time);

#endif
