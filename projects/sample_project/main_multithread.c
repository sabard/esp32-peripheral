// orchestration of multiple instances of the same task (task array) happpening in parallel contrlled by 'c'

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define BLINK_GPIO 2
#define BLINK_PRIORITY  (tskIDLE_PRIORITY+2)
#define WATCH_PRIORITY (tskIDLE_PRIORITY+1)
#define N_BLINK_TASKS 10
#define QUEUE_SIZE 10


static const char *TAG = "example";
char input;
int number;
static uint8_t s_led_state=0;
static int inactive_tasks[N_BLINK_TASKS];
QueueHandle_t task_queue[N_BLINK_TASKS];
static int avail_task = -1;
int task_index;
int t;

void vTaskWatch( void *pvParameters )
{
	for ( ;; )
	{
		input = getchar();
		if (input == 'c')
		{
            avail_task = -1;
            for (task_index=0; task_index<N_BLINK_TASKS; task_index++) { // what is the next avail free-floating task 
                if (inactive_tasks[task_index] >= 0) { // if this task index is an inactive task
                    avail_task = task_index;
                }
            }
            
            printf("\n");
            for (task_index=0; task_index<N_BLINK_TASKS; task_index++) {  
                printf(" ,%d, ", inactive_tasks[task_index]);
            }
            printf("\n");
            printf("\n next available task using: %d\n", avail_task);

            if (avail_task < 0) {
                ESP_LOGE(TAG, "There are no more avail tasks to run");
            }
            else {
			    xQueueSend( task_queue[avail_task], &avail_task, portMAX_DELAY );
            }
		}
		vTaskDelay( 10 / portTICK_PERIOD_MS );
	}

}

void blink(int t) {
            inactive_tasks[t] = -1;// take it off the inactive task array
            s_led_state = 1;
			printf("TASK %d is turning LED SWITCH ON\n",t);
		    vTaskDelay( 2000 / portTICK_PERIOD_MS );
			s_led_state=0;
            printf("TASK %d is turning LED SWITCH OFF\n",t);
            inactive_tasks[t] = t; // take it back on the inactive task array
}

void vTaskBlink( void *pvParameters ) // it's as if these are all the same one task but it's just recalled again and again to kick off the function mulitple times with different inputs
{
    vTaskSetThreadLocalStoragePointer(NULL, 0, (void *) t); // keep the memory of the task's task index since t itself is a global variable that is shared and changes
    int index = (int) pvTaskGetThreadLocalStoragePointer(NULL, 0);
    for ( ;; )
	{
		if ( xQueueReceive( task_queue[index], &index, portMAX_DELAY) == pdTRUE )  
		{  
            printf("\nTASK %d ACTIVATED\n", index);
            //printf("INDEX IS %d AT THE START\n", index);
            blink(index);	
            //printf("INDEX IS %d AT THE END\n", index);
            printf("TASK %d DEACTIVATED\n", index);
		}
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
                
	xTaskCreate(vTaskWatch, "WATCH", 4096, NULL, WATCH_PRIORITY, NULL);
    for (t=0; t<N_BLINK_TASKS; t++) { // for each task in the blink task array
        inactive_tasks[t] = t; //initialize list of inactive tasks
        if (!(task_queue[t] = xQueueCreate(QUEUE_SIZE, sizeof(t)))) {
                printf("Could not create queue.");
        }
        xTaskCreate(vTaskBlink, NULL, 4096, &t, BLINK_PRIORITY, NULL);
    
    }
	
}
