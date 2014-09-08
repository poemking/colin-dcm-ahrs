#ifndef __AHRS_H
#define __AHRS_H

typedef struct {
	float roll_angle;
	float pitch_angle;
	float yaw_angle;
} attitude_t;

void gyro_integrate(attitude_t *attitude, vector3d_f_t gyro_scaled_data,
	float period_time);

#endif
