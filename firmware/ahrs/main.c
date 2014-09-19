#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "usart.h"
#include "i2c.h"
#include "led.h"
#include "timer.h"

#include "mpu6050.h"

#include "delay.h"

#include "ahrs_task.h"
#include "telemetry_task.h"

int main()
{
	/* Peripheral initialization */
	led_init();
	debug_port_init();
	usart3_init(57600);
	i2c1_init();
	timer2_init();

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
