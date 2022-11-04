#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define BLINK_PRIORITY  (tskIDLE_PRIORITY + 2)

static const char* TAG="MyModule";

void vTaskBlink( void * pvParameters )
{
	printf("BLINK\n");
	printf("BLINK 2\n");
	printf("BLINK 3 \n");	
	while (1)
	{
		printf("BLINK 4 \n");	
		vTaskDelay(1000000);
	}
	//for ( ;; ) 
	//{
//		printf("BLINK\n");
//	}
}


void app_main(void)
{	//creating tasks and their handles  
	TaskHandle_t xTaskBlink = NULL;
	printf("task create start");
	xTaskCreate(vTaskBlink, "BLINK", 20000, NULL , BLINK_PRIORITY , &xTaskBlink );
	printf("task create end");
	//configASSERT( xTaskBlink );
	if ( xTaskBlink != NULL)
	{
		printf("DELETING TASK");
		vTaskDelete( xTaskBlink );
	}

}
