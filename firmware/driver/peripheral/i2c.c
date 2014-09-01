#include "stm32f4xx_conf.h"

void i2c1_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_OD,
		.GPIO_PuPd = GPIO_PuPd_UP	
	};
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	I2C_InitTypeDef I2C_InitStruct = {
		.I2C_ClockSpeed = 400000,
		.I2C_Mode = I2C_Mode_I2C,
		.I2C_DutyCycle = I2C_DutyCycle_2,
		.I2C_OwnAddress1 = 0x00,
		.I2C_Ack = I2C_Ack_Disable,
		.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit	
	};
	I2C_Init(I2C1, &I2C_InitStruct);

	I2C_Cmd(I2C1, ENABLE);
}

void i2c_read_start(I2C_TypeDef* i2c_channel, uint8_t device_address)
{
	while(I2C_GetFlagStatus(i2c_channel, I2C_FLAG_BUSY));

	I2C_GenerateSTART(i2c_channel, ENABLE);

	while(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(i2c_channel, device_address, I2C_Direction_Receiver);

	while(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));	
}

void i2c_write_start(I2C_TypeDef* i2c_channel, uint8_t device_address)
{
	while(I2C_GetFlagStatus(i2c_channel, I2C_FLAG_BUSY));

	I2C_GenerateSTART(i2c_channel, ENABLE);

	while(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(i2c_channel, device_address, I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
}

void i2c_stop(I2C_TypeDef* i2c_channel)
{
	I2C_GenerateSTOP(i2c_channel, ENABLE);
}

void i2c_write(I2C_TypeDef* i2c_channel, uint8_t data)
{
	I2C_SendData(i2c_channel, data);

	while(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t i2c_read_ack(I2C_TypeDef* i2c_channel)
{
	I2C_AcknowledgeConfig(i2c_channel, ENABLE);

	while(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_BYTE_RECEIVED));

	return I2C_ReceiveData(i2c_channel);
}

uint8_t i2c_read_nack(I2C_TypeDef* i2c_channel)
{
	I2C_AcknowledgeConfig(i2c_channel, DISABLE);

	I2C_GenerateSTOP(i2c_channel, ENABLE);

	while(!I2C_CheckEvent(i2c_channel, I2C_EVENT_MASTER_BYTE_RECEIVED));

	return I2C_ReceiveData(i2c_channel);
}

uint8_t i2c_single_read(I2C_TypeDef* i2c_channel, uint8_t device_address,
	uint8_t register_address)
{
	uint8_t data;

	/* Send the register address */
	i2c_write_start(i2c_channel, device_address);
	i2c_write(i2c_channel, register_address);
	i2c_stop(i2c_channel);

	/* Receive the data */
	i2c_read_start(i2c_channel, device_address);
	data = i2c_read_nack(i2c_channel);
	i2c_stop(i2c_channel);

	return data;
}

void i2c_single_write(I2C_TypeDef* i2c_channel, uint8_t device_address,
	uint8_t register_address, uint8_t data)
{
	/* Send the register address */
	i2c_write_start(i2c_channel, device_address);
	i2c_write(i2c_channel, register_address);
	i2c_stop(i2c_channel);

	/* Write the data into the register */
	i2c_write_start(i2c_channel, device_address);
	i2c_write(i2c_channel, data);
	i2c_stop(i2c_channel);
}
