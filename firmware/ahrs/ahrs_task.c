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

#include "ahrs.h"

//The pre-filter times count is depend on EMA filter's alpha value
#define IMU_EMA_PREFILTER_CNT 200

imu_data_t imu_data;
ahrs_data_t ahrs_data;

extern SemaphoreHandle_t ahrs_task_semaphore;

void ahrs_task()
{
	/* AHRS initialization */
	vector3d_f_t accel_ema_last_data, gyro_ema_last_data; //buffer of EMA last data

	/* Prepare the first EMA filter data */
	mpu6050_read_unscaled_data(&imu_data.accel_unscaled_data, &imu_data.gyro_unscaled_data);
	mpu6050_fix_bias(&imu_data.accel_unscaled_data, &imu_data.gyro_unscaled_data);
	mpu6050_accel_convert_to_scale(&imu_data.accel_unscaled_data, &accel_ema_last_data);
	mpu6050_gyro_convert_to_scale(&imu_data.gyro_unscaled_data, &gyro_ema_last_data);

	/* Pre-filter the IMU data */
	int i;
	for(i = 0; i < IMU_EMA_PREFILTER_CNT; i++) {
		/* New sampling data */
		mpu6050_read_unscaled_data(&imu_data.accel_unscaled_data, &imu_data.gyro_unscaled_data);
		mpu6050_fix_bias(&imu_data.accel_unscaled_data, &imu_data.gyro_unscaled_data);
		mpu6050_accel_convert_to_scale(&imu_data.accel_unscaled_data, &imu_data.accel_raw_data);
		mpu6050_gyro_convert_to_scale(&imu_data.gyro_unscaled_data, &imu_data.gyro_raw_data);

		/* Filter the data with EMA filter (Make the filter stable) */
		vector3d_exponential_moving_average(imu_data.accel_raw_data, &accel_ema_last_data,
			&imu_data.accel_filtered_data, 0.01725);
		vector3d_exponential_moving_average(imu_data.gyro_raw_data, &gyro_ema_last_data,
			&imu_data.gyro_filtered_data, 0.01725);
	}

	/* Initialize the gyroscope euler angle with accelerometer */
	accel_estimate_euler_angle(&ahrs_data.accel_attitude, imu_data.accel_filtered_data);
	ahrs_data.gyro_attitude = ahrs_data.accel_attitude;

	/* AHRS main procedure */
	while(1) {
		while(xSemaphoreTake(ahrs_task_semaphore, 1) == pdFALSE);

		led_off(LED2); //Turn off the LED before calculating the AHRS information
		debug_port_off(DEBUG_PORT);

		/* Get the new IMU unscaled raw data */
		mpu6050_read_unscaled_data(&imu_data.accel_unscaled_data, &imu_data.gyro_unscaled_data);

		/* Fix the sensor bias */
		mpu6050_fix_bias(&imu_data.accel_unscaled_data, &imu_data.gyro_unscaled_data);

		/* Scale the IMU raw data */
		mpu6050_accel_convert_to_scale(&imu_data.accel_unscaled_data, &imu_data.accel_raw_data);
		mpu6050_gyro_convert_to_scale(&imu_data.gyro_unscaled_data, &imu_data.gyro_raw_data);

		/* filter the IMU raw data (Exponential Moving Average filter) */
		vector3d_exponential_moving_average(imu_data.accel_raw_data, &accel_ema_last_data,
			&imu_data.accel_filtered_data, 0.01725);
		vector3d_exponential_moving_average(imu_data.gyro_raw_data, &gyro_ema_last_data,
			&imu_data.gyro_filtered_data, 0.01725);

		/* Attitude estimation */
		//Get the euler angle from accelerometer by taking gravity as reference
		accel_estimate_euler_angle(&ahrs_data.accel_attitude, imu_data.accel_filtered_data);
		//Get the euler angle from gyroscope by integrate the angle velocity
		gyro_integrate(&ahrs_data.gyro_attitude, imu_data.gyro_filtered_data, 0.002); //500hz, period = 0.02

		/* Take accelerometer as reference to fix the gyroscope integrate error accumulation */
		gyro_error_eliminate(&ahrs_data.gyro_attitude, ahrs_data.accel_attitude, 0.175,
			imu_data.accel_filtered_data, 0.0015);

		led_on(LED2); //Turn on the LED after calculating the AHRS information
		debug_port_on(DEBUG_PORT);

		vTaskDelay(1);
	}
}

void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(ahrs_task_semaphore, &xHigherPriorityTaskWoken);

		/* Clear the timer counter */
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
	}
}
