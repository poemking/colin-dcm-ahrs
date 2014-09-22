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

extern imu_data_t imu_data;
extern ahrs_data_t ahrs_data;

void usart_plot_task()
{
	while(1) {
		led_off(LED3); //Turn off the LED before the trasmission

		uint8_t payload[256] = {'\0'}; //About 64 float variable
		int payload_count = 0;

		/* Convert the onboard parameter to the byte */
		//Accelerometer raw data
		payload_count += convert_vector3d_float_to_byte(&imu_data.accel_raw_data, payload + payload_count);
		//Accelerometer filtered data
		payload_count += convert_vector3d_float_to_byte(&imu_data.accel_filtered_data, payload + payload_count);
		//Gyroscope raw data
		payload_count += convert_vector3d_float_to_byte(&imu_data.gyro_raw_data, payload + payload_count);
		//Gyroscope filtered data
		payload_count += convert_vector3d_float_to_byte(&imu_data.gyro_filtered_data, payload + payload_count);
		//Attitude Data (from accelerometer)
		payload_count += convert_attitude_to_byte(&ahrs_data.accel_attitude, payload + payload_count);
		//Attitude data (from gyroscope)
		payload_count += convert_attitude_to_byte(&ahrs_data.gyro_attitude, payload + payload_count);

		/* Send the onboard parameter */
		send_onboard_parameter(payload, payload_count);

		led_on(LED3); //Turn on the LED after the transmission

		vTaskDelay(10);
	}
}
