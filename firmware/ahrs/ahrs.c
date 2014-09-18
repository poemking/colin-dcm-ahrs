#include <math.h>

#include "vector_space.h"
#include "ahrs.h"

void gyro_integrate(attitude_t *attitude, vector3d_f_t gyro_scaled_data,
	float period_time)
{
	/* Convert the body frame(sensor frame) back to the inertial frame */
	vector3d_f_t iframe_gyro;
	//x' = x + cos(roll)tan(pitch)z + sin(roll)tan(pitch)y
	iframe_gyro.x = 
		gyro_scaled_data.x +
		cos(attitude->roll_angle) * tan(attitude->pitch_angle) * gyro_scaled_data.z +
		sin(attitude->roll_angle) * tan(attitude->pitch_angle) * gyro_scaled_data.y;
	//y' = cos(pitch)y - sin(roll)z
	iframe_gyro.y =
		cos(attitude->pitch_angle) * gyro_scaled_data.y -
		sin(attitude->roll_angle) * gyro_scaled_data.z;
	//z' = cos(roll)sec(pitch)z + sin(roll)sec(pitch)y
	iframe_gyro.z =
		cos(attitude->roll_angle) * (1 / cos(attitude->pitch_angle)) * gyro_scaled_data.z +
		sin(attitude->roll_angle) * (1 / cos(attitude->pitch_angle)) * gyro_scaled_data.y;

	/* Intergrate the angle velocity */
	attitude->roll_angle += iframe_gyro.x * period_time;
	attitude->pitch_angle += iframe_gyro.y * period_time;
	attitude->yaw_angle += iframe_gyro.z * period_time;
}
