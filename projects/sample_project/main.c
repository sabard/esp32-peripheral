#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define BLINK_GPIO 2
#define BLINK_PRIORITY  (tskIDLE_PRIORITY + 2)
#define WATCH_PRIORITY  (tskIDLE_PRIORTIY + 1)

static const char* TAG="MyModule";

//static uint8_t s_led_state = 0;

//static char input;

void vTaskBlink(void *pvParameters)
{
	for ( ;; ) 
	{
		// if xTaskGenericNotify when making this blocked by completion of another task 
		
		ESP_LOGI(TAG, "BLINK\n");

		//ESP_LOGI(TAG, "change LED\n");
		//gpio_set_level(BLINK_GPIO, s_led_state);
		//s_led_state = !s_led_state;		
	}
	//vTaskDelete( NULL ); 
}

//void vTaskWatch(void *pvParameters)
//{
//	for ( ;; )
//	{
//		while (1)
//		{
//			input=getchar();
//			if (input == 'b') 
//			{
				//trigger task to blink
//			}
//			
//		}
//	}
//}

//static void configure_led(void)
//{
//	gpio_reset_pin(BLINK_GPIO);
//	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
//}

void app_main(void)
{	//creating tasks and their handles  
	//TaskHandle_t xTaskWatch = NULL;
	TaskHandle_t xTaskBlink = NULL;
	//xTaskCreate(vTaskWatch, "WATCH", STACK_SIZE,NULL,WATCH_PRIORITY,&xTaskWatch);
	ESP_LOGI(TAG, "task create next line");
	xTaskCreate(vTaskBlink, "BLINK", configMINIMAL_STACK_SIZE , NULL , BLINK_PRIORITY , &xTaskBlink );
	ESP_LOGI(TAG, "task create prev line");
	// starting main task 
	//configure_led();
	//ESP_LOGI(TAG, "hello \n");
	configASSERT( xTaskBlink );
	if ( xTaskBlink != NULL)
	{
		ESP_LOGI(TAG, "DELETING TASK");
		vTaskDelete( xTaskBlink );
	}

	//while (1) {
		//ESP_LOGI(TAG, "squidward %s", s_led_state == true ? "ON" : "OFF");
		//blink_led();
		//s_led_state = !s_led_state;
		//vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
	//	input = getchar()
	//	if (input == 'b') {
	//		blink_led();
	//	}
	//}
}
