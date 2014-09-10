#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "usart.h"
#include "i2c.h"
#include "led.h"

#include "mpu6050.h"
#include "telemetry.h"

#include "delay.h"
#include "vector_space.h"
#include "moving_average.h"

#define IMU_SMA_SAMPLING_CNT 100

/* IMU unscaled data */
vector3d_16_t accel_unscaled_data, gyro_unscaled_data; //IMU unscaled data

/* IMU scaled data */
vector3d_f_t accel_raw_data, gyro_raw_data; //IMU raw data
vector3d_f_t accel_filtered_data, gyro_filtered_data; //IMU filtered data

void ahrs_task()
{
	vector3d_f_t accel_sma_fifo[IMU_SMA_SAMPLING_CNT];
	vector3d_f_t gyro_sma_fifo[IMU_SMA_SAMPLING_CNT];

	/* Fill the FIFO of the SMA filter */
	int i;
	for(i = 0; i < IMU_SMA_SAMPLING_CNT; i++) {
		/* Get the new sampling data */
		mpu6050_read_unscaled_data(&accel_unscaled_data, &gyro_unscaled_data);

		/* Fix the sensor bias */
		mpu6050_fix_bias(&accel_unscaled_data, &gyro_unscaled_data);

		mpu6050_accel_convert_to_scale(&accel_unscaled_data, &accel_raw_data);
		mpu6050_gyro_convert_to_scale(&gyro_unscaled_data, &gyro_raw_data);

		/* Push the new sampling data into the FIFO */
		accel_sma_fifo[i].x = accel_raw_data.x;
		accel_sma_fifo[i].y = accel_raw_data.y;
		accel_sma_fifo[i].z = accel_raw_data.z;
		gyro_sma_fifo[i].x = gyro_raw_data.x;
		gyro_sma_fifo[i].y = gyro_raw_data.y;
		gyro_sma_fifo[i].z = gyro_raw_data.z;
	}	

	while(1) {
		/* Get the new IMU unscaled raw data */
		mpu6050_read_unscaled_data(&accel_unscaled_data, &gyro_unscaled_data);

		/* Fix the sensor bias */
		mpu6050_fix_bias(&accel_unscaled_data, &gyro_unscaled_data);

		/* Scale the IMU raw data */
		mpu6050_accel_convert_to_scale(&accel_unscaled_data, &accel_raw_data);
		mpu6050_gyro_convert_to_scale(&gyro_unscaled_data, &gyro_raw_data);

		/* filter the IMU raw data (SMA filter) */
		vector3d_simple_moving_average(accel_raw_data, accel_sma_fifo,
			&accel_filtered_data, IMU_SMA_SAMPLING_CNT);
		vector3d_simple_moving_average(gyro_raw_data, gyro_sma_fifo,
			&gyro_filtered_data, IMU_SMA_SAMPLING_CNT);

		vTaskDelay(1);
	}
}

void usart_plot_task()
{
	while(1) {
		led_off(LED2); //Turn off the LED before the trasmission

		uint8_t payload[256] = {'\0'}; //About 64 float variable
		int payload_count = 0;

		/* Convert the onboard parameter to the byte */
		//Accelerator raw data
		payload_count += convert_vector3d_float_to_byte(&accel_raw_data, payload + payload_count);
		//Gyroscope raw data
		payload_count += convert_vector3d_float_to_byte(&gyro_raw_data, payload + payload_count);

		/* Send the onboard parameter */
		send_onboard_parameter(payload, payload_count);

		led_on(LED2); //Turn on the LED after the transmission

		vTaskDelay(1);
	}
}

int main()
{
	/* Peripheral initialization */
	led_init();
	usart3_init(57600);
	i2c1_init();

	//Make sure all the peripheral is finished the initialization
	delay_ms(1000);

	/* Device initialization */
	while(mpu6050_init());

	led_on(LED1); //Initialization is finished

	/* Task creation */
	//Attitude and Heading Reference System (AHRS) task
	xTaskCreate(ahrs_task, (portCHAR *)"AHRS task",
		4096, NULL, tskIDLE_PRIORITY + 2, NULL);

	//USART plot task
	xTaskCreate(usart_plot_task, (portCHAR *)"USART plot task",
		2048, NULL, tskIDLE_PRIORITY + 1, NULL);


	/* Start schedule */
	vTaskStartScheduler();

	return 0;
}
