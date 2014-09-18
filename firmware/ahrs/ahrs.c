#include <math.h>

#include "vector_space.h"
#include "ahrs.h"

#define deg_to_rad(angle) angle * 0.0174532925f //Angle * (PI / 180)

void gyro_integrate(attitude_t *attitude, vector3d_f_t gyro_scaled_data,
	float period_time)
{
	/* Convert the body frame(sensor frame) back to the inertial frame */
	vector3d_f_t iframe_gyro;
	//x' = x + cos(roll)tan(pitch)z + sin(roll)tan(pitch)y
	iframe_gyro.x = 
		gyro_scaled_data.x +
		cos(deg_to_rad(attitude->roll_angle)) * tan(deg_to_rad(attitude->pitch_angle)) * gyro_scaled_data.z +
		sin(deg_to_rad(attitude->roll_angle)) * tan(deg_to_rad(attitude->pitch_angle)) * gyro_scaled_data.y;
	//y' = cos(pitch)y - sin(roll)z
	iframe_gyro.y =
		cos(deg_to_rad(attitude->pitch_angle)) * gyro_scaled_data.y -
		sin(deg_to_rad(attitude->roll_angle)) * gyro_scaled_data.z;
	//z' = cos(roll)sec(pitch)z + sin(roll)sec(pitch)y
	iframe_gyro.z =
		cos(deg_to_rad(attitude->roll_angle)) * (1 / cos(deg_to_rad(attitude->pitch_angle))) * gyro_scaled_data.z +
		sin(deg_to_rad(attitude->roll_angle)) * (1 / cos(deg_to_rad(attitude->pitch_angle))) * gyro_scaled_data.y;

	/* Intergrate the angle velocity */
	attitude->roll_angle += iframe_gyro.x * period_time;
	attitude->pitch_angle += iframe_gyro.y * period_time;
	attitude->yaw_angle += iframe_gyro.z * period_time;
}
