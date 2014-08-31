#include "stm32f4xx_conf.h"
#include "i2c.h"

#define MPU6050_DEVICE_ADDRESS 0x68 << 1

uint8_t mpu6050_read_who_am_i()
{
	uint8_t who_am_i_data = 0;

	i2c_write_start(I2C1, MPU6050_DEVICE_ADDRESS);
	i2c_write(I2C1, 0x75);
	i2c_stop(I2C1);

	i2c_read_start(I2C1, MPU6050_DEVICE_ADDRESS);
	who_am_i_data = i2c_read_nack(I2C1);
	i2c_stop(I2C1);

	return who_am_i_data;
}
