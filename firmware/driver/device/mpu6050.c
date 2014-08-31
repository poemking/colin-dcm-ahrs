#include "stm32f4xx_conf.h"
#include "i2c.h"

#define MPU6050_DEVICE_ADDRESS 0x68

/* I2C Timeout exception */
typedef enum {I2C_SUCCESS, I2C_TIMEOUT} I2C_Status;
int i2c_timeout;
I2C_Status eeprom_i2c_status;
#define I2C_TIMED(x) i2c_timeout = 0xFFFF; eeprom_i2c_status = I2C_SUCCESS; \
while(x) { if(i2c_timeout-- == 0) { eeprom_i2c_status = I2C_TIMEOUT; goto i2c_restart;} }

uint8_t mpu6050_read_who_am_i()
{

	if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		return 0;

	uint8_t who_am_i_data = 0;

	/* I2C start signal */
	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send the device address as slave address */
	I2C_Send7bitAddress(I2C1, MPU6050_DEVICE_ADDRESS, I2C_Direction_Transmitter);

	/* Transmit the WHO_AM_I address */
	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, 0x75);

	I2C_TIMED(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Receive the WHO_AM_I data with NO ACK Message */
	I2C_AcknowledgeConfig(I2C1, DISABLE);

	I2C_TIMED(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

	who_am_i_data = I2C_ReceiveData(I2C1);

	/* I2C stop signal */
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_TIMED(I2C1->CR1 & I2C_CR1_STOP);	

	i2c_restart:
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_AcknowledgeConfig(I2C1, ENABLE);

	return who_am_i_data;
}
