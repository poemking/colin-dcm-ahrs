#include <string.h>

#include "stm32f4xx_conf.h"
#include "i2c.h"

#include "delay.h"

#include "vector_space.h"

/* MPU6050 register address */
#define MPU6050_DEVICE_ADDRESS 0x68 << 1

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

/* MPU6050 scale value */
#define MPU6050A_2g 0.0000610352f
#define MPU6050A_4g 0.0001220703f
#define MPU6050A_8g 0.0002441406f
#define MPU6050A_16g 0.0004882813f

#define MPU6050G_250dps 0.007633587786f
#define MPU6050G_500dps 0.015267175572f
#define MPU6050G_1000dps 0.030487804878f
#define MPU6050G_2000dps 0.060975609756f

#define MPU6050_ACCEL_SCALE MPU6050A_4g
#define MPU6050_GYRO_SCALE MPU6050G_2000dps

uint8_t mpu6050_read_who_am_i()
{
	return i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, 0x75);
}

void mpu6050_wakeup()
{
	i2c_single_write(I2C1, MPU6050_DEVICE_ADDRESS,  MPU6050_PWR_MGMT_1, 0x00);

	delay_ms(1000);
}

int mpu6050_init()
{
	/* Check MPU6050 device is alive or not */
	if(mpu6050_read_who_am_i() != 0x68) return 1;

	//Wakeup the device
	mpu6050_wakeup();

	//MPU6050 accelerator : +-4g mode
	i2c_single_write(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_CONFIG, 0x08);
	//MPU6050 gyroscope : +-2000dps mode
	i2c_single_write(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);

	return 0;
}

void mpu6050_read_raw_data(vector3d_16_t *accel_raw_data, vector3d_16_t *gyro_raw_data)
{
	uint8_t buffer[12]; //12 for 6 axis high/low byte

	buffer[0] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_XOUT_L); 
	buffer[1] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_XOUT_H);
	buffer[2] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_YOUT_L);
	buffer[3] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_YOUT_H);
	buffer[4] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_ZOUT_L);
	buffer[5] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_ZOUT_H);
	buffer[6] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_XOUT_L);
	buffer[7] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_XOUT_H);
	buffer[8] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_YOUT_L);
	buffer[9] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_YOUT_H);
	buffer[10] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_ZOUT_L);
	buffer[11] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_ZOUT_H);

	memcpy(&accel_raw_data->x, &buffer[0], sizeof(int16_t));
	memcpy(&accel_raw_data->y, &buffer[2], sizeof(int16_t));
	memcpy(&accel_raw_data->z, &buffer[4], sizeof(int16_t));
	memcpy(&gyro_raw_data->x, &buffer[6], sizeof(int16_t));
	memcpy(&gyro_raw_data->y, &buffer[8], sizeof(int16_t));
	memcpy(&gyro_raw_data->z, &buffer[10], sizeof(int16_t));
}

int32_t mpu6050_convert_scale_to_acceleration(int16_t accel_scale)
{
	return accel_scale * MPU6050_ACCEL_SCALE;
}

int32_t mpu6050_convert_scale_to_angle_velocity(int16_t gyro_scale)
{
	return gyro_scale * MPU6050_GYRO_SCALE;
}
