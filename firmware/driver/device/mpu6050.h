#ifndef __MPU6050_H
#define __MPU6050_H

uint8_t mpu6050_read_who_am_i();

int mpu6050_init();

void mpu6050_read_raw_data(vector3d_16_t *accel_raw_data, vector3d_16_t *gyro_raw_data);

int32_t mpu6050_convert_scale_to_acceleration(int16_t accel_scale);
int32_t mpu6050_convert_scale_to_angle_velocity(int16_t gyro_scale);


#endif
