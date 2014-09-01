#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "usart.h"
#include "i2c.h"

#include "vector_space.h"

#include "mpu6050.h"

void ahrs_task()
{
	vector3d_16_t accel_raw_data, gyro_raw_data;

	while(1) {
		mpu6050_read_raw_data(&accel_raw_data, &gyro_raw_data);
	}
}

void usart_plot_task()
{
	while(1);
}

int main()
{
	/* Hardware initialization */
	usart3_init(57600);
	i2c1_init();
	while(mpu6050_init());

	/* Task creation */
	xTaskCreate(ahrs_task, (portCHAR *)"AHRS task",
		512, NULL, tskIDLE_PRIORITY + 2, NULL);

		/* Task creation */
	xTaskCreate(ahrs_task, (portCHAR *)"USART plot task",
		512, NULL, tskIDLE_PRIORITY + 1, NULL);


	/* Start schedule */
	vTaskStartScheduler();

	return 0;
}
