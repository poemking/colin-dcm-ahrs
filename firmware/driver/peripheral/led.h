#ifndef __LED_H
#define __LED_H

#define led_toggle(led)  GPIO_ToggleBits(led)
#define led_off(led)  GPIO_ResetBits(led)
#define led_on(led)  GPIO_SetBits(led)

#define LED1 GPIOG, GPIO_Pin_13
#define LED2 GPIOG, GPIO_Pin_14

void led_init();

#endif
