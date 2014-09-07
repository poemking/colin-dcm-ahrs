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

static void mpu6050_read(uint8_t *data, uint8_t register_adress, int data_count)
{
	i2c_read(I2C1, MPU6050_DEVICE_ADDRESS, register_adress, data, data_count);
}

static void mpu6050_write(uint8_t data)
{
	i2c_single_write(I2C1, MPU6050_DEVICE_ADDRESS, register_address, data);
}

uint8_t mpu6050_read_who_am_i()
{
	uint8_t data;
	i2c_read(I2C1, MPU6050_DEVICE_ADDRESS, 0x75, &data, 1);

	return data;
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

	delay_ms(1000);

	/* Calibrate the device */
	//mpu6050_gyro_calibrate();

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
		buffer[0] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_XOUT_L);
		buffer[1] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_XOUT_H);
		buffer[2] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_YOUT_L);
		buffer[3] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_YOUT_H);
		buffer[4] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_ZOUT_L);
		buffer[5] = i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_ZOUT_H);

		memcpy(&gyro_new_sampling_data.x, &buffer[0], sizeof(int16_t));
		memcpy(&gyro_new_sampling_data.y, &buffer[2], sizeof(int16_t));
		memcpy(&gyro_new_sampling_data.z, &buffer[4], sizeof(int16_t));

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
