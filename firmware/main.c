#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "usart.h"
#include "i2c.h"

#include "mpu6050.h"

void ahrs_task()
{
	while(1) {
		uint8_t mpu6050_test_data = mpu6050_read_who_am_i();
		printf("MPU6050 Who am I: %d\n", mpu6050_test_data);
		vTaskDelay(200);
	}
}

int main()
{
	/* Hardware initialization */
	usart3_init(57600);
	i2c1_init();
	while(mpu6050_init());

	/* Task creation */
	xTaskCreate(ahrs_task, (portCHAR *)"AHRS task",
		512, NULL, tskIDLE_PRIORITY + 1, NULL);

	/* Start schedule */
	vTaskStartScheduler();

	return 0;
}
