#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "usart.h"
#include "led.h"

#include "telemetry.h"

#include "vector_space.h"

#include "ahrs.h"

extern ahrs_data_t ahrs_data;

extern vector3d_f_t accel_raw_data, gyro_raw_data;
extern vector3d_f_t accel_ema_filter_data, gyro_ema_filter_data;

void usart_plot_task()
{
	while(1) {
		led_off(LED3); //Turn off the LED before the trasmission

		uint8_t payload[256] = {'\0'}; //About 64 float variable
		int payload_count = 0;

		/* Convert the onboard parameter to the byte */
		//Accelerator raw data
		payload_count += convert_vector3d_float_to_byte(&accel_raw_data, payload + payload_count);
		//Accelerator filter data
		payload_count += convert_vector3d_float_to_byte(&accel_ema_filter_data, payload + payload_count);
		//Gyroscope raw data
		payload_count += convert_vector3d_float_to_byte(&gyro_raw_data, payload + payload_count);
		//Gyroscope filter data
		payload_count += convert_vector3d_float_to_byte(&gyro_ema_filter_data, payload + payload_count);
		//Attitude(from gyroscope)
		payload_count += convert_attitude_to_byte(&ahrs_data.gyro_attitude, payload + payload_count);

		/* Send the onboard parameter */
		send_onboard_parameter(payload, payload_count);

		led_on(LED3); //Turn on the LED after the transmission

		vTaskDelay(10);
	}
}
