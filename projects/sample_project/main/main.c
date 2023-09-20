// controlling the LED by pressing 'c' on the keyboard with binary semaphores
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define BLINK_GPIO 2
#define BLINK_PRIORITY  (tskIDLE_PRIORITY+2)
#define WATCH_PRIORITY (tskIDLE_PRIORITY+1)
#define N_BLINK_TASKS 3

char input;
int number;
static uint8_t s_led_state=0;
SemaphoreHandle_t xSemaphore = NULL;

void vTaskWatch( void *pvParameters )
{
	for ( ;; )
	{
		input = getchar();
		if (input == 'c')
		{
			xSemaphoreGive( xSemaphore );
		}
		vTaskDelay( 10 / portTICK_PERIOD_MS );
	}

}


void vTaskBlink( void *pvParameters )
{
	for ( ;; )
	{
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE )
		{
			printf("LED SWITCH ON\n");
		    vTaskDelay( 2000 / portTICK_PERIOD_MS );
			s_led_state=!s_led_state;
            printf("LED SWITCH OFF\n");
			
		}
	}
	
}

typedef void (*task_type)(void *); // declare a type of definition as a pointer to a function that takes in no arguments and returns nothing
task_type task_array[N_BLINK_TASKS]; // make an array of pointers to tasks or functions  
int i;


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
    for (int i=0; i<N_BLINK_TASKS; i++) { // for each task pointer
        task_array[i] = &vTaskBlink;
        printf("\n %p \n", &vTaskBlink);
        int x = sizeof(&vTaskBlink);
        printf("\n %d \n", x);
        //memcpy(task_array[i], &vTaskBlink, sizeof(&vTaskBlink));
                
        //void (*task_array[i])(void *pvParameters) { // declare the value of the ith task pointer as a void with the routines below
        //	for ( ;; ) {
        //		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE )
        //		{
        //			printf("LED SWITCH ON\n");
        //		    vTaskDelay( 2000 / portTICK_PERIOD_MS );
        //			s_led_state=!s_led_state;
        //            printf("LED SWITCH OFF\n");
        //			
        //		}
        //	}
        //    
        //}
    }

	if (xSemaphore !=NULL)
	{
		// creating tasks and their handles  
		//TaskHandle_t xTaskBlink = NULL;
		TaskHandle_t xTaskWatch = NULL;
		//xTaskCreate(vTaskBlink, "BLINK", 4096, NULL , BLINK_PRIORITY , &xTaskBlink);
		xTaskCreate(vTaskWatch, "WATCH", 4096, NULL, WATCH_PRIORITY, &xTaskWatch);

        for (int i=0; i<N_BLINK_TASKS; i++) { // for each task in the blink task array
            xTaskCreate(*task_array[i], NULL, 4096, NULL, BLINK_PRIORITY, NULL);
        }
	}
	else
	{
		printf("semaphore did not create sucessfully\n");
	}
	
}
