#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "usart.h"
#include "i2c.h"

void ahrs_task()
{
	while(1);
}

int main()
{
	/* Hardware initialization */
	usart3_init(57600);
	i2c1_init();

	/* Task creation */
	xTaskCreate(ahrs_task, (portCHAR *)"AHRS task",
		512, NULL, tskIDLE_PRIORITY + 1, NULL);

	/* Start schedule */
	vTaskStartScheduler();

	return 0;
}
