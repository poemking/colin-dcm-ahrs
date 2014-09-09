#include <string.h>

#include "stm32f4xx_conf.h"
#include "i2c.h"

#include "mpu6050.h"

#include "delay.h"

#include "vector_space.h"

#define MPU6050_ACCEL_SCALE MPU6050A_4g
#define MPU6050_GYRO_SCALE MPU6050G_2000dps

#define GYRO_SAMPLING_COUNT 10000

vector3d_16_t mpu6050_gyro_offset;

static void mpu6050_read(uint8_t register_adress, uint8_t *data, int data_count)
{
	while(i2c_read(I2C1, MPU6050_DEVICE_ADDRESS, register_adress, data, data_count)
		== I2C_TIMEOUT);
}

static void mpu6050_write(uint8_t register_address, uint8_t data)
{
	while(i2c_write(I2C1, MPU6050_DEVICE_ADDRESS, register_address, data) == I2C_TIMEOUT);
}

uint8_t mpu6050_read_who_am_i()
{
	uint8_t data;
	i2c_read(I2C1, MPU6050_DEVICE_ADDRESS, 0x75, &data, 1);

	return data;
}

void mpu6050_reset()
{
	mpu6050_write(MPU6050_PWR_MGMT_1, 0x80);
	
	delay_ms(1000);
}

void mpu6050_wakeup()
{
	mpu6050_write(MPU6050_PWR_MGMT_1, 0x00);

	delay_ms(1000);
}

int mpu6050_init()
{
	/* Check MPU6050 device is alive or not */
	if(mpu6050_read_who_am_i() != 0x68) return 1;

	//Reset the device
	mpu6050_reset();

	//Wakeup the device
	mpu6050_wakeup();

	//MPU6050 accelerator : +-4g mode
	mpu6050_write(MPU6050_ACCEL_CONFIG, 0x08);
	//MPU6050 gyroscope : +-2000dps mode
	mpu6050_write(MPU6050_GYRO_CONFIG, 0x18);

	delay_ms(1000);

	/* Calibrate the device */
	mpu6050_gyro_calibrate();

	return 0;
}

void mpu6050_gyro_calibrate()
{
	uint8_t buffer[6];

	vector3d_16_t gyro_average_cache;
	vector3d_16_t gyro_new_sampling_data;

	/* Calculate the bias of the gyroscope */
	int i;
	for(i = 0; i < GYRO_SAMPLING_COUNT; i++) {
		/* Get the new gyro data */
		mpu6050_read(MPU6050_GYRO_XOUT_H, buffer, 6);

		gyro_new_sampling_data.x = (buffer[0] << 8) | buffer[1];
		gyro_new_sampling_data.y = (buffer[2] << 8) | buffer[3];
		gyro_new_sampling_data.z = (buffer[4] << 8) | buffer[5];
	
		/* Add cache data with new data */
		gyro_average_cache.x += gyro_new_sampling_data.x;
		gyro_average_cache.y += gyro_new_sampling_data.y;
		gyro_average_cache.z += gyro_new_sampling_data.z;
	}

	/* Get the average of all samping datas */
	mpu6050_gyro_offset.x = gyro_average_cache.x / GYRO_SAMPLING_COUNT;	
	mpu6050_gyro_offset.y = gyro_average_cache.y / GYRO_SAMPLING_COUNT;
	mpu6050_gyro_offset.z = gyro_average_cache.z / GYRO_SAMPLING_COUNT;
}

void mpu6050_read_unscaled_data(vector3d_16_t *accel_raw_data, vector3d_16_t *gyro_raw_data)
{
	uint8_t buffer[14]; //12 for 6 axis high/low byte

	/* Get the new data */
	mpu6050_read(MPU6050_ACCEL_XOUT_H, buffer, 14); 

	accel_raw_data->x = (buffer[0] << 8) | buffer[1];
	accel_raw_data->y = (buffer[2] << 8) | buffer[3];
	accel_raw_data->z = (buffer[4] << 8) | buffer[5];
	gyro_raw_data->x = (buffer[8] << 8) | buffer[9];
	gyro_raw_data->y = (buffer[10] << 8) | buffer[11];
	gyro_raw_data->z = (buffer[12] << 8) | buffer[13];
}

void mpu6050_accel_convert_to_scale(vector3d_16_t *accel_unscaled_data,
	vector3d_f_t *accel_scaled_data)
{
	 accel_scaled_data->x = accel_unscaled_data->x * MPU6050_ACCEL_SCALE;
	 accel_scaled_data->y = accel_unscaled_data->y * MPU6050_ACCEL_SCALE;
	 accel_scaled_data->z = accel_unscaled_data->z * MPU6050_ACCEL_SCALE;
}

void mpu6050_gyro_convert_to_scale(vector3d_16_t *gyro_unscaled_data,
	vector3d_f_t *gyro_scaled_data)
{
	 gyro_scaled_data->x = gyro_unscaled_data->x * MPU6050_GYRO_SCALE;
	 gyro_scaled_data->y = gyro_unscaled_data->y * MPU6050_GYRO_SCALE;
	 gyro_scaled_data->z = gyro_unscaled_data->z * MPU6050_GYRO_SCALE;
}
