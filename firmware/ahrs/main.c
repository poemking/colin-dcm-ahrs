#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "usart.h"
#include "i2c.h"
#include "led.h"

#include "delay.h"

#include "vector_space.h"

#include "mpu6050.h"

/* IMU scaled data */
vector3d_16_t accel_scaled_data, gyro_scaled_data; //The scale data from the IMU

/* IMU unscaled data */
vector3d_f_t accel_raw_data, gyro_raw_data; //The raw data from the IMU
vector3d_f_t accel_filt_data, gyro_filt_data; //The imu data filting by filter

void ahrs_task()
{
	while(1) {
		mpu6050_read_raw_data(&accel_scaled_data, &gyro_scaled_data);

		/* Unscale the IMU data */
		mpu6050_convert_scale_to_acceleration(&accel_scaled_data, &accel_raw_data);
		mpu6050_convert_scale_to_angle_velocity(&gyro_scaled_data, &gyro_raw_data);

		vTaskDelay(1);
	}
}

void usart_plot_task()
{
	while(1) {
		led_on(LED2);

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
		512, NULL, tskIDLE_PRIORITY + 2, NULL);

	//USART plot task
	xTaskCreate(usart_plot_task, (portCHAR *)"USART plot task",
		512, NULL, tskIDLE_PRIORITY + 1, NULL);


	/* Start schedule */
	vTaskStartScheduler();

	return 0;
}
