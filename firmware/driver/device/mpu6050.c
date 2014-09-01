#include "stm32f4xx_conf.h"
#include "i2c.h"

#define MPU6050_DEVICE_ADDRESS 0x68 << 1

uint8_t mpu6050_read_who_am_i()
{
	return i2c_single_read(I2C1, MPU6050_DEVICE_ADDRESS, 0x75);
}
