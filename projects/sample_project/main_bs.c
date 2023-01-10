// controlling the LED by pressing 'c' on the keyboard with binary semaphores
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define BLINK_GPIO 2
#define BLINK_PRIORITY  (tskIDLE_PRIORITY+2)
#define WATCH_PRIORITY (tskIDLE_PRIORITY+1)

char input;
int number;
static uint8_t s_led_state=0;
SemaphoreHandle_t xSemaphore = NULL;

vid vTaskWatch( void *pvParameters )
{
	for ( ;; )
	{
		input = getchar();
		if (input == 'c')
		{
			xSemaphoreGive( xSemaphore );
		}
		printf("end of for loop -- watch\n");
		vTaskDelay( 10 / portTICK_PERIOD_MS );
	}

}

void vTaskBlink( void * pvParameters )
{
	for ( ;; )
	{
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE )
		{
			printf("LED SWITCH\n");
			gpio_set_level(BLINK_GPIO, s_led_state);
			s_led_state=!s_led_state;
			
		}
		printf("end of loop -- blink\n");
	}
	
}

static void configure_led(void)
{
      gpio_reset_pin(BLINK_GPIO);
      gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}


void app_main(void)
{	
	configure_led();
	
	// creating semaphore
	xSemaphore = xSemaphoreCreateBinary();

	if (xSemaphore !=NULL)
	{
		// creating tasks and their handles  
		TaskHandle_t xTaskBlink = NULL;
		TaskHandle_t xTaskWatch = NULL;
		xTaskCreate(vTaskBlink, "BLINK", 4096, NULL , BLINK_PRIORITY , &xTaskBlink);
		xTaskCreate(vTaskWatch, "WATCH", 4096, NULL, WATCH_PRIORITY, &xTaskWatch);

	}
	else
	{
		printf("semaphore did not create sucessfully\n");
	}
	
}
