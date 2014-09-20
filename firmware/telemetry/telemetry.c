#include <stdint.h>
#include <string.h>

#include "usart.h"

#include "vector_space.h"

#include "ahrs.h"

int convert_float_to_byte(float *float_data, uint8_t *byte_data)
{
	memcpy(byte_data, (uint8_t *)float_data, sizeof(float));

	return sizeof(float);
}

int convert_vector3d_float_to_byte(vector3d_f_t *vector_data, uint8_t *byte_data)
{
	memcpy(byte_data, (uint8_t *)vector_data, sizeof(vector3d_f_t));

	return sizeof(vector3d_f_t);
}

int convert_attitude_to_byte(attitude_t *attitude_data, uint8_t *byte_data)
{
	memcpy(byte_data, (uint8_t *)attitude_data, sizeof(attitude_t));

	return sizeof(attitude_t);
}

static uint8_t generate_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = 0;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

void send_onboard_parameter(uint8_t *payload, int payload_count)
{
	uint8_t checksum;

	checksum = generate_checksum_byte(payload, payload_count);

	usart3_putc('@'); //Send the start byte
	usart3_putc((uint8_t)payload_count); //Send the payload count

	//Send the payload data
	int i;
	for(i = 0; i < payload_count; i++)
		usart3_putc(payload[i]);

	usart3_putc(checksum); //Send the checksum data
}
