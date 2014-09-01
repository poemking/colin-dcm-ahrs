#ifndef __I2C_H
#define __I2C_H

void i2c1_init();

void i2c_read_start(I2C_TypeDef* i2c_channel, uint8_t device_address);
void i2c_write_start(I2C_TypeDef* i2c_channel, uint8_t device_address);
void i2c_stop(I2C_TypeDef* i2c_channel);
void i2c_write(I2C_TypeDef* i2c_channel, uint8_t data);
uint8_t i2c_read_ack(I2C_TypeDef* i2c_channel);
uint8_t i2c_read_nack(I2C_TypeDef* i2c_channel);

uint8_t i2c_single_read(I2C_TypeDef* i2c_channel, uint8_t device_address,
	uint8_t register_address);
void i2c_single_write(I2C_TypeDef* i2c_channel, uint8_t device_address,
	uint8_t register_address, uint8_t data);

#endif
