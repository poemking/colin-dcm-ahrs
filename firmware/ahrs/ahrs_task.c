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

#define IMU_SMA_SAMPLING_CNT 400

#define USE_SMA_FILTER 0
#define USE_WMA_FILTER 1
#define USE_EMA_FILTER 2

#define IMU_FILTER USE_EMA_FILTER

/* IMU unscaled data */
vector3d_16_t accel_unscaled_data, gyro_unscaled_data;

/* IMU scaled data */
vector3d_f_t accel_raw_data, gyro_raw_data;
/* Filter data */
#if IMU_FILTER == USE_SMA_FILTER
vector3d_f_t accel_sma_filter_data, gyro_sma_filter_data;
#endif
#if IMU_FILTER == USE_WMA_FILTER
vector3d_f_t accel_wma_filter_data, gyro_wma_filter_data;
#endif
#if IMU_FILTER == USE_EMA_FILTER
vector3d_f_t accel_ema_filter_data, gyro_ema_filter_data;
#endif

void ahrs_task()
{
	#if (IMU_FILTER == USE_SMA_DATA) || (IMU_FILTER == USE_WMA_FILTER)
	vector3d_f_t accel_moving_average_fifo[IMU_SMA_SAMPLING_CNT];
	vector3d_f_t gyro_moving_average_fifo[IMU_SMA_SAMPLING_CNT];
	#elif IMU_FILTER == USE_EMA_FILTER
	vector3d_f_t accel_ema_last_data, gyro_ema_last_data;
	#endif

	/* Prepare the Moving Average filter data */
	int i;
	for(i = 0; i < IMU_SMA_SAMPLING_CNT; i++) {
		#if (IMU_FILTER == USE_SMA_DATA) || (IMU_FILTER == USE_WMA_FILTER)
		/* Get the new sampling data */
		mpu6050_read_unscaled_data(&accel_unscaled_data, &gyro_unscaled_data);

		/* Fix the sensor bias */
		mpu6050_fix_bias(&accel_unscaled_data, &gyro_unscaled_data);

		mpu6050_accel_convert_to_scale(&accel_unscaled_data, &accel_raw_data);
		mpu6050_gyro_convert_to_scale(&gyro_unscaled_data, &gyro_raw_data);

		/* Prepare the Moving Average FIFO (SMA, WMA) */
		accel_moving_average_fifo[i].x = accel_raw_data.x;
		accel_moving_average_fifo[i].y = accel_raw_data.y;
		accel_moving_average_fifo[i].z = accel_raw_data.z;
		gyro_moving_average_fifo[i].x = gyro_raw_data.x;
		gyro_moving_average_fifo[i].y = gyro_raw_data.y;
		gyro_moving_average_fifo[i].z = gyro_raw_data.z;
		#endif

		#if IMU_FILTER == USE_EMA_FILTER
		/* Filter the data with EMA filter (Make the filter stable) */
		vector3d_exponential_moving_average(accel_raw_data, &accel_ema_last_data,
			&accel_ema_filter_data, 0.01725);
		vector3d_exponential_moving_average(gyro_raw_data, &gyro_ema_last_data,
			&gyro_ema_filter_data, 0.01725);
		#endif
	}	

	while(1) {
		led_off(LED2); //Turn off the LED before calculating the AHRS information
		debug_port_off(DEBUG_PORT);

		/* Get the new IMU unscaled raw data */
		mpu6050_read_unscaled_data(&accel_unscaled_data, &gyro_unscaled_data);

		/* Fix the sensor bias */
		mpu6050_fix_bias(&accel_unscaled_data, &gyro_unscaled_data);

		/* Scale the IMU raw data */
		mpu6050_accel_convert_to_scale(&accel_unscaled_data, &accel_raw_data);
		mpu6050_gyro_convert_to_scale(&gyro_unscaled_data, &gyro_raw_data);

		#if IMU_FILTER == USE_SMA_FILTER
		/* filter the IMU raw data (Simple Moving Average filter) */
		vector3d_simple_moving_average(accel_raw_data, accel_moving_average_fifo,
			&accel_sma_filter_data, IMU_SMA_SAMPLING_CNT);
		vector3d_simple_moving_average(gyro_raw_data, gyro_moving_average_fifo,
			&gyro_sma_filter_data, IMU_SMA_SAMPLING_CNT);
		#endif

		#if IMU_FILTER == USE_WMA_FILTER
		/* filter the IMU raw data (Weight Moving Average filter) */
		vector3d_weight_moving_average(accel_raw_data, accel_moving_average_fifo,
			&accel_wma_filter_data, IMU_SMA_SAMPLING_CNT);
		vector3d_weight_moving_average(gyro_raw_data, gyro_moving_average_fifo,
			&gyro_wma_filter_data, IMU_SMA_SAMPLING_CNT);
		#endif

		#if IMU_FILTER == USE_EMA_FILTER
		/* filter the IMU raw data (Exponential Moving Average filter) */
		vector3d_exponential_moving_average(accel_raw_data, &accel_ema_last_data,
			&accel_ema_filter_data, 0.01725);
		vector3d_exponential_moving_average(gyro_raw_data, &gyro_ema_last_data,
			&gyro_ema_filter_data, 0.01725);
		#endif

		led_on(LED2); //Turn on the LED after calculating the AHRS information
		debug_port_on(DEBUG_PORT);

		vTaskDelay(1);
	}
}
