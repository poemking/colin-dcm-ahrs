#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include "vector_space.h"

#include "ahrs.h"

int convert_float_to_byte(float *float_data, uint8_t *byte_data);
int convert_vector3d_float_to_byte(vector3d_f_t *vector_data, uint8_t *byte_data);
int convert_attitude_to_byte(attitude_t *attitude_data, uint8_t *byte_data);

void send_onboard_parameter(uint8_t *payload, int payload_count);

#endif
