#include "vector_space.h"
#include "ahrs.h"

void gyro_integrate(attitude_t *attitude, vector3d_f_t gyro_scaled_data,
	float period_time)
{
	attitude->roll_angle += gyro_scaled_data.x * period_time;
	attitude->pitch_angle += gyro_scaled_data.y * period_time;
	attitude->yaw_angle += gyro_scaled_data.z * period_time;
}
