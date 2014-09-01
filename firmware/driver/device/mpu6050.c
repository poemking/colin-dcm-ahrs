#include "stm32f4xx_conf.h"
#include "i2c.h"

/* MPU6050 register address */
#define MPU6050_DEVICE_ADDRESS 0x68 << 1
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

uint8_t mpu6050_read_who_am_i()
{
	return i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, 0x75);
}

int mpu6050_init()
{
	/* Check MPU6050 device is alive or not */
	if(mpu6050_read_who_am_i() != 0x68) return 1;

	//MPU6050 gyroscope : +-2000dps mode
	i2c_single_write(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);
	//MPU6050 accelerator : +-4g mode
	i2c_single_write(I2C1, MPU6050_DEVICE_ADDRESS, MPU6050_ACCEL_CONFIG, 0x08);

	return 0;
}
