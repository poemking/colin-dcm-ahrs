#ifndef __I2C_H
#define __I2C_H

typedef enum {I2C_SUCCESS, I2C_TIMEOUT} I2C_Status;

void i2c1_init();

uint8_t i2c_single_read(I2C_TypeDef* i2c_channel, uint8_t device_address,
	uint8_t register_address);
void i2c_single_write(I2C_TypeDef* i2c_channel, uint8_t device_address,
	uint8_t register_address, uint8_t data);

I2C_Status i2c_read(I2C_TypeDef* i2c_channel, uint8_t device_address, uint8_t register_address, uint8_t *data,
	int data_count);
I2C_Status i2c_write(I2C_TypeDef* i2c_channel, uint8_t device_address, uint8_t register_address,
	uint8_t data);

#endif
